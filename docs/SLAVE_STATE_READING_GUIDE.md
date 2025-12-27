# 从站状态读取完整指南

## 快速答案

**从站状态如何读取？**

```cpp
// 方式 1：直接访问内存（我们的做法）
uint16 state = soem_context_.slavelist[1].state;
// → 从缓存中读取，快速但略有延迟 (~10ms)

// 方式 2：通过函数调用（更安全）
uint16 state = lifter.read_slave_state(1);
// → 包含验证和错误检查
// → 返回: 0x0001(INIT), 0x0002(PREOP), 0x0004(SAFEOP), 0x0008(OP)
```

---

## 深度解析

### 从站状态在 SOEM 中的位置

```
SOEM 网络上下文 (ecx_contextt soem_context_)
│
└─ slavelist[128]  ← 数组：最多 128 个从站
   │
   ├─ slavelist[0]  (主站，通常不用)
   │  ├─ state
   │  ├─ ALstatuscode
   │  └─ ...
   │
   ├─ slavelist[1]  ← ★ 我们要读的
   │  ├─ state = 0x0002  ← 【这个字段】
   │  ├─ ALstatuscode = 0x0000
   │  ├─ name[33] = "Sine Drive"
   │  ├─ eep_man = 0x000000A1
   │  ├─ eep_id = 0xA0000001
   │  ├─ SM[0..3]  (SyncManager 配置)
   │  └─ ... (80+ 个其他字段)
   │
   └─ slavelist[2..127]  (未使用)
```

### 四步读取过程

```
【1】检查初始化        if (!soem_initialized_) return 0;
     ├─ 确保 SOEM 网络已初始化
     └─ 确保 slavelist[] 已被填充

【2】验证从站号        if (slave < 1 || slave > slavecount) return 0;
     ├─ slave 必须 >= 1（不能是 0）
     └─ slave 必须 <= slavecount（不能超出范围）

【3】读取状态值        uint16 state = slavelist[slave].state;
     ├─ 纯内存读取（不涉及 EtherCAT 网络）
     ├─ 速度快：< 1 微秒
     ├─ 不阻塞
     └─ 延迟：~10ms（上次定时器更新以来）

【4】返回结果         return state;
     └─ 0x0001/0x0002/0x0004/0x0008/0x0010
```

### 状态值的含义

| 值 | 十进制 | 名称 | 含义 | 什么时候 |
|----|------|------|------|---------|
| 0x0001 | 1 | INIT | 从站刚启动，未配置 | ecx_init() 之后 |
| 0x0002 | 2 | PREOP | 从站已知，等待同步 | ecx_config_init() 自动转换 |
| 0x0004 | 4 | SAFEOP | 从站同步，可接收 PDO | ecx_statecheck(..., SAFEOP) 后 |
| 0x0008 | 8 | OP | 从站运行，交换数据 | ecx_statecheck(..., OP) 后 |
| 0x0010 | 16 | ERROR | 从站故障 | 从站内部错误或掉线 |

### 状态何时更新？

```
【初始化时】
  ecx_init()          → slavelist[].state 仍为空
  ↓
  ecx_config_init()   → slavelist[1].state = 0x0002 (PREOP)
  ↓
  ecx_statecheck()    → slavelist[1].state = 0x0004 (SAFEOP)

【运行时】每 10ms (定时器周期):
  timer_callback():
    ecx_send_processdata(&soem_context_);
    ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);
    ↓
    SOEM 自动读取所有从站的状态并更新 slavelist[].state
    
  → 如果从站正常运行，state 保持不变
  → 如果从站掉线，state 变为 ERROR (0x0010)
```

### 状态转换流程图

```
┌──────────────────────────────────────────────────┐
│ 从站启动 / 连接                                    │
└────────────────┬─────────────────────────────────┘
                 │ ecx_init() 自动
                 ▼
         ┌──────────────┐
         │    INIT      │ (0x0001)
         │   初始化中    │
         └──────┬───────┘
                │ ecx_config_init() 自动
                ▼
         ┌──────────────┐
         │    PREOP     │ (0x0002)
         │  配置已完成  │
         └──────┬───────┘
                │ ecx_statecheck(..., SAFEOP)
                ▼
         ┌──────────────┐
         │   SAFEOP     │ (0x0004)
         │  同步已完成  │
         └──────┬───────┘
                │ ecx_statecheck(..., OP)
                ▼
         ┌──────────────┐
         │      OP      │ (0x0008)
         │    运行中    │
         └──────┬───────┘
                │ 从站故障/断线
                ▼
         ┌──────────────┐
         │    ERROR     │ (0x0010)
         │    故障      │
         └──────────────┘
```

---

## 代码级别的理解

### C 语言角度（SOEM 源代码中）

```c
// 在 soem/ec_main.h 中定义
typedef struct {
    uint16 state;           // ← 这是我们读的字段
    uint16 ALstatuscode;
    // ... 80+ 个其他字段
    char name[EC_MAXNAME+1];
} ec_slavet;

// 在网络轮询函数中
void ecx_receive_processdata(ecx_contextt *context) {
    for (uint16 slave = 1; slave <= context->slavecount; slave++) {
        // 从 EtherCAT 网络读取从站的状态寄存器（物理读）
        uint16 network_state = read_from_network(slave, 0x130);
        
        // 更新缓存
        context->slavelist[slave].state = network_state;
    }
}
```

### C++ 角度（我们的代码中）

```cpp
uint16 Lifter::read_slave_state(uint16 slave) {
    // ① 检查初始化
    if (!soem_initialized_) return 0;
    
    // ② 检查范围
    if (slave < 1 || slave > soem_context_.slavecount) return 0;
    
    // ③ 读取缓存值（【关键】不涉及网络通信）
    uint16 state = soem_context_.slavelist[slave].state;
    //              ↑              ↑          ↑
    //         SOEM 上下文    从站数组   状态字段
    
    // ④ 返回
    return state;
}
```

### 内存地址角度

```
假设 soem_context_ 基地址 = 0x1000

soem_context_.slavecount    @ 0x1000
soem_context_.slavelist     @ 0x1010

slavelist[1]                @ 0x1010 + (1 * sizeof(ec_slavet))
                            @ 0x1010 + (1 * 512)  ← ec_slavet 大约 512 字节
                            @ 0x1210

slavelist[1].state          @ 0x1210 + 0  ← state 是结构体的第一个成员
                            @ 0x1210

slavelist[1].ALstatuscode   @ 0x1210 + 2  ← 紧跟在 state 之后
slavelist[1].name           @ 0x1210 + X  ← 在结构体的某个偏移位置
```

---

## 实际日志示例

```
运行节点时的日志输出：

[INFO] SOEM ecx_init succeeded
[INFO] EtherCAT slaves found: 1
[INFO] SOEM initialization completed successfully
[INFO] Dumping 1 EtherCAT slaves:
[INFO]  Slave 1: name=Sine Drive
[INFO] === Demonstrating slave state management ===
[INFO] Slave 1 (Sine Drive) current state: 0x0001
       ↑     ↑                  ↑           ↑
     从站号  从站名称          消息         状态值 (INIT)
       |                                    |
       └─ read_slave_state(1) 返回的信息 ──┘

[INFO] Attempting to set slave 1 to OPERATIONAL state...
[INFO] Setting slave 1 (Sine Drive) from state 0x0001 to state 0x0008
[WARN] Slave 1 state transition: requested 0x0008, got 0x0002 instead.
                                 请求 OP         转到了 PREOP
                                 (0x0008)        (0x0002)
```

---

## 常见问题解答

### Q: 为什么 read_slave_state() 不直接查询网络？
**A:** 
- 直接查询需要 SDO（Service Data Object）通信，耗时 10-100ms
- 我们用的缓存方式是 SOEM 的标准做法，延迟 < 10ms
- 适合实时轮询（定时器周期就是 10ms）

### Q: slavelist[0] 是什么？
**A:** 
- slavelist[0] 代表主站（EtherCAT 主机自己）
- 在从站状态查询中通常不用
- 我们的代码强制 slave >= 1

### Q: 如何检测从站掉线？
**A:** 
```cpp
uint16 state = read_slave_state(1);
if (state == 0x0010) {  // EC_STATE_ERROR
    RCLCPP_ERROR(..., "Slave 1 lost!");
}
```

### Q: ALstatuscode 是什么？
**A:** 
- EtherCAT 错误代码，由从站设备报告
- 常见值：
  - 0x0000 = OK
  - 0x0021 = Invalid input configuration
  - 0x0023 = Watchdog timed out
- 在 verify_slave_state() 中可以看到

---

## 总结

| 方面 | 描述 |
|------|------|
| **读取位置** | `soem_context_.slavelist[slave].state` |
| **数据类型** | uint16 (16 位无符号整数) |
| **更新方式** | SOEM 定时轮询自动更新 (~10ms) |
| **读取速度** | < 1 微秒（纯内存读取） |
| **阻塞性** | 非阻塞 |
| **状态值** | 0x0001(INIT) / 0x0002(PREOP) / 0x0004(SAFEOP) / 0x0008(OP) / 0x0010(ERROR) |
| **验证方法** | 通过 verify_slave_state() 检查 |
| **错误代码** | 见 slavelist[slave].ALstatuscode |
