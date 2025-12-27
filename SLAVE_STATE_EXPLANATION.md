# EtherCAT 从站状态读取机制详解

## 📋 从站状态读取的三个步骤

### 1. **初始化检查**
```cpp
if (!soem_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot read slave state");
    return 0;
}
```
- 确保 SOEM 网络已初始化（在 `initialize_soem()` 中设置 `soem_initialized_ = true`）
- 如果未初始化，则无法读取从站状态（网络不存在）
- 返回 0（表示无效状态）

### 2. **从站编号有效性检查**
```cpp
if (slave < 1 || slave > soem_context_.slavecount) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid slave number %u (valid range: 1-%d)", 
                 slave, soem_context_.slavecount);
    return 0;
}
```
- `soem_context_.slavecount` — 网络中扫描到的从站总数
- 从站编号范围：1 到 slavecount（从 1 开始计数，不是从 0）
- 例如：如果扫描到 1 个从站，则只能访问编号 1 的从站

### 3. **读取状态值**
```cpp
uint16 state = soem_context_.slavelist[slave].state;
```

**关键数据结构：**
```c
// SOEM 中的从站数组（在 soem_context 中）
ec_slavet slavelist[EC_MAXSLAVE];  // EC_MAXSLAVE = 最多128个从站

// ec_slavet 结构中包含：
typedef struct ec_slave {
    uint16 state;           // ← 我们读取的就是这个字段
    uint16 ALstatuscode;    // EtherCAT 错误代码
    uint16 configadr;       // 从站配置地址
    uint32 eep_man;         // 制造商 ID
    uint32 eep_id;          // 产品编号
    char name[EC_MAXNAME+1];// 从站名称（"Sine Drive" 等）
    // ... 还有 80+ 个其他字段
} ec_slavet;
```

## 🔢 从站状态值及其含义

EtherCAT 定义了 4 种主要状态，使用 16 位编码：

| 状态名称 | 十六进制值 | 十进制值 | 含义 |
|---------|-----------|--------|------|
| INIT (初始化) | 0x0001 | 1 | 从站刚启动或被重置，还未配置 |
| PREOP (准就绪) | 0x0002 | 2 | 从站已配置，但尚未同步 |
| SAFEOP (安全操作) | 0x0004 | 4 | 从站同步，可以接收 PDO，但未进入运行模式 |
| OP (运行) | 0x0008 | 8 | 从站完全就绪，正常交换 PDO 数据 |

**示例日志解读：**
```
[INFO] Slave 1 (Sine Drive) current state: 0x0001
```
→ 从站 1 目前处于 **INIT** 状态（刚初始化）

```
[WARN] Slave 1 state transition: requested 0x0008, got 0x0002 instead
```
→ 请求转到 OP (0x0008)，但实际只到了 PREOP (0x0002)

## 🔄 从站状态转换流程

```
┌─────────┐
│  INIT   │  (0x0001) — 从站启动
└────┬────┘
     │ (ecx_config_init 自动转换)
     ▼
┌─────────┐
│ PREOP   │  (0x0002) — 从站已知
└────┬────┘
     │ (用户或 ecx_statecheck 请求)
     ▼
┌─────────┐
│ SAFEOP  │  (0x0004) — 从站同步
└────┬────┘
     │ (用户或 ecx_statecheck 请求)
     ▼
┌─────────┐
│   OP    │  (0x0008) — 从站运行
└─────────┘
```

## 📊 数据读取的实际过程

### 什么时候状态被更新？

1. **SOEM 网络轮询时** — `ecx_send_processdata()` 和 `ecx_receive_processdata()`
   ```cpp
   ecx_send_processdata(&soem_context_);
   ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);
   ```
   - 这两个函数每 10ms 被调用（在 `timer_callback()` 中）
   - **每次轮询时，SOEM 会读取所有从站的当前状态并更新 `slavelist[].state`**

2. **显式状态检查时** — `ecx_statecheck()`
   ```cpp
   ecx_statecheck(&soem_context_, slave, target_state, EC_TIMEOUTSTATE);
   ```
   - 直接查询从站状态并尝试转换到目标状态
   - 阻塞等待状态转换完成（最多 EC_TIMEOUTSTATE = 2000ms）

### 我们的 `read_slave_state()` 读取的是什么？

```cpp
uint16 state = soem_context_.slavelist[slave].state;
```

- **读取的是内存中缓存的状态值**，不是实时查询
- 这个值由 SOEM 在最近一次网络轮询时更新
- 通常延迟 < 10ms（因为定时器回调每 10ms 执行一次）

## 🔍 完整流程示例

**初始化时：**
```
1. ecx_init(&soem_context_, "enp86s0")
   → SOEM 初始化网络接口
   → slavelist[].state 会被设置但值仍为 0（未扫描）

2. ecx_config_init(&soem_context_)
   → SOEM 自动扫描所有从站
   → 发送 INIT → PREOP 状态转换命令
   → slavelist[1].state = 0x0002 (PREOP)
   → slavelist[1].name = "Sine Drive"
   → soem_context_.slavecount = 1

3. ecx_statecheck(&soem_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)
   → 向从站 0 (master) 发送 SAFE_OP 转换
   → 所有从站跟随转换到 SAFE_OP
   → slavelist[1].state = 0x0004 (SAFEOP)
```

**运行时：**
```
每 10ms (timer_callback):
  ecx_send_processdata(&soem_context_);
  ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);
  
  → 读取从站的实时状态并更新 slavelist[].state
  → 如果从站掉线或异常，state 会改变
```

**调用 `read_slave_state(1)` 时：**
```cpp
// 1. 检查初始化标志
if (!soem_initialized_) return 0;  ✓ OK

// 2. 检查从站编号
if (slave < 1 || slave > 1) return 0;  ✓ OK (slave=1, count=1)

// 3. 从缓存读取状态
uint16 state = soem_context_.slavelist[1].state;
// state = 0x0002 (PREOP) 或 0x0004 (SAFEOP) 或 0x0008 (OP)

// 4. 打印并返回
RCLCPP_INFO(..., "Slave 1 (Sine Drive) current state: 0x0004");
return 0x0004;
```

## 📌 关键要点总结

| 概念 | 说明 |
|-----|------|
| **状态存储位置** | `soem_context_.slavelist[slave].state` (16位无符号整数) |
| **更新方式** | SOEM 网络轮询时自动更新（无需手动查询） |
| **读取延迟** | 典型 < 10ms（取决于定时器周期） |
| **从站编号** | 1 到 slavecount（从 1 开始） |
| **无效编号返回值** | 0（表示错误或无效） |
| **状态值范围** | 0x0001(INIT) → 0x0002(PREOP) → 0x0004(SAFEOP) → 0x0008(OP) |
| **状态转换方式** | `ecx_statecheck()` 或网络轮询自动转换 |

## 💡 实用代码片段

### 定期监测从站状态
```cpp
// 在定时器回调中
void monitor_slave_state() {
    uint16 state = read_slave_state(1);
    
    // 状态值判断
    switch(state) {
        case 0x0001: RCLCPP_INFO(..., "INIT"); break;
        case 0x0002: RCLCPP_INFO(..., "PREOP"); break;
        case 0x0004: RCLCPP_INFO(..., "SAFEOP"); break;
        case 0x0008: RCLCPP_INFO(..., "OP"); break;
        default:     RCLCPP_ERROR(..., "Unknown state 0x%04x", state);
    }
}
```

### 等待从站达到目标状态
```cpp
bool wait_for_state(uint16 slave, uint16 target_state, int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        
        uint16 state = read_slave_state(slave);
        if (state == target_state) return true;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_ERROR(..., "Timeout waiting for slave state");
    return false;
}
```

## 🔧 SOEM 源代码参考

**状态定义** (在 `soem/ec_main.h` 中)：
```c
#define EC_STATE_INIT       0x01
#define EC_STATE_PREOP      0x02  
#define EC_STATE_BOOT       0x03
#define EC_STATE_SAFEOP     0x04
#define EC_STATE_OPERATIONAL 0x08
#define EC_STATE_ERROR      0x10
```

**状态读取** (SOEM 内部轮询)：
```c
// 在 ecx_receive_processdata() 中调用
// SOEM 读取每个从站的状态寄存器（0x130 地址）
// 并更新 slavelist[i].state
```
