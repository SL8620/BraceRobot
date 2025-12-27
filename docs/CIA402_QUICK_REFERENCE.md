# CiA402 快速参考卡

## 🔌 PDO 映射速查

### RxPDO 输出 (主站 → 从站)
| 字节 | 对象代码 | 名称 | API 函数 |
|-----|---------|------|---------|
| 0-1 | 0x6040 | Control Word | `set_motor_control_word(uint16_t)` |
| 2-5 | 0x607A | Target Position | `write_motor_target_position(double)` |
| 6-9 | 0x6081 | Profile Velocity | `set_motor_velocity(int32_t)` |
| 10 | 0x6060 | Modes of Operation | `set_motor_mode(enum)` |
| 11-12 | 0x6071 | Target Torque | *(预留)* |
| 13-16 | 0x60FF | Target Velocity | *(预留)* |
| 17-20 | 0x60B2 | Acceleration | `set_motor_acceleration(uint32_t)` |

### TxPDO 输入 (从站 → 主站)
| 字节 | 对象代码 | 名称 | API 函数 |
|-----|---------|------|---------|
| 0-1 | 0x6041 | Status Word | `read_motor_status_word()` |
| 2-5 | 0x6064 | Position Actual | `read_motor_position()` |
| 6-9 | 0x606C | Velocity Actual | `read_motor_velocity()` |
| 10 | 0x6061 | Mode Display | *(自动读取)* |
| 11-12 | 0x603F | Error Code | `read_motor_error_code()` |
| 13-14 | 0x6077 | Torque Actual | `read_motor_torque()` |
| 15-18 | 0x60FD | Digital Inputs | *(预留)* |

---

## 🎮 常用控制字值

| 值 | 十进制 | 含义 | 说明 |
|----|-------|------|------|
| `0x000F` | 15 | **使能** | 所有控制位 = 1，电机完全使能 |
| `0x0000` | 0 | **禁用** | 所有控制位 = 0，电机停止 |
| `0x0007` | 7 | **快速停止** | 清除位 2（Quick Stop），立即停止 |
| `0x804F` | 32847 | **故障复位** | 设置位 6 脉冲清除故障 |

### 控制字位含义
```
bit 0: Shutdown          (0=禁用, 1=使能)
bit 1: Enable Operation  (0=禁用, 1=使能)
bit 2: Quick Stop        (0=快停, 1=允许)
bit 3: Enable Voltage    (0=禁用, 1=使能)
bit 6: Fault Reset       (脉冲清除故障)
```

---

## 🔄 工作模式速查

| 模式值 | 名称 | 说明 | 何时使用 |
|--------|------|------|---------|
| **1** | **Profiled Position** | 轮廓位置（速度/加速度自动规划） | ✅ **电缸推荐** |
| 3 | Velocity | 直接速度模式 | 恒速运动 |
| 4 | Profiled Torque | 扭矩模式 | 恒力运动 |
| 8 | Cyclic Sync Position | 实时同步位置 | 高精度实时控制 |

---

## ⚡ 电机使能流程（3 步）

```cpp
// 1️⃣ 使能电机
lifter.set_motor_control_word(0x000F);

// 2️⃣ 设置工作模式
lifter.set_motor_mode(cia402::MODE_PROFILED_POSITION);

// 3️⃣ 配置速度和加速度
lifter.set_motor_velocity(100);      // RPM
lifter.set_motor_acceleration(200);  // rpm/s
```

---

## 📍 位置控制（最常用）

### 读取位置
```cpp
double pos = lifter.read_motor_position();  // 毫米
```

### 设置目标位置
```cpp
lifter.write_motor_target_position(500.0);  // 500 mm
// 驱动器自动以配置的速度/加速度运动
```

### 位置偏移（原点校准）
```cpp
// 如果机械原点在 50mm，设置偏移使其显示为 0mm
lifter.set_motor_position_offset(50.0);

// 读取当前偏移
double offset = lifter.get_motor_position_offset();
```

---

## 📊 实时反馈读取

### 状态字
```cpp
cia402::StatusWord status = lifter.read_motor_status_word();

// 便利方法
if (status.is_operation_enabled()) { }      // 电机使能？
if (status.is_fault()) { }                  // 有故障？
if (status.is_voltage_enabled()) { }        // 电压使能？
if (status.is_quick_stop_active()) { }      // 快速停止？

// 获取当前状态名称
printf("Status: %s\n", status.get_state_name());
// 输出: "Operation Enabled" / "Fault" / "Not Ready to Switch On" 等
```

### 实时数据
```cpp
int32_t vel = lifter.read_motor_velocity();        // RPM
int16_t torque = lifter.read_motor_torque();       // 0.1% FT
uint16_t error = lifter.read_motor_error_code();   // 错误码
```

---

## 🛑 故障处理

### 检查错误
```cpp
uint16_t error = lifter.read_motor_error_code();

if (error != 0) {
    RCLCPP_ERROR(logger, "Motor error: 0x%04x", error);
}
```

### 常见错误码
| 错误码 | 含义 |
|--------|------|
| `0x0000` | ✅ 无错误 |
| `0x1000` | ❌ 通用错误 |
| `0x2100` | ❌ 过流 |
| `0x2200` | ❌ 电压异常 |
| `0x2300` | ❌ 温度异常 |
| `0x8110` | ❌ 通信故障 |

### 故障复位
```cpp
// 方式 1: 完全禁用后重新使能
lifter.set_motor_control_word(0x0000);
std::this_thread::sleep_for(std::chrono::milliseconds(100));
lifter.set_motor_control_word(0x000F);

// 方式 2: 设置故障复位位（脉冲）
lifter.set_motor_control_word(0x804F);
```

---

## 📈 监测电机运动（示例）

```cpp
// 设置目标位置后，监测运动
lifter.write_motor_target_position(500.0);

for (int i = 0; i < 50; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    double pos = lifter.read_motor_position();
    int32_t vel = lifter.read_motor_velocity();
    auto status = lifter.read_motor_status_word();
    
    printf("[%d] Pos: %.2f mm, Vel: %d rpm, Status: %s\n",
           i, pos, vel, status.get_state_name());
    
    if (status.is_fault()) {
        printf("ERROR: Motor fault!\n");
        break;
    }
}
```

---

## ⚙️ 自定义参数

### 改变脉冲转换因子

编码器分辨率不同时需要调整。例如 4000 脉冲/mm：

**在 `lifter.cpp` 中修改：**

```cpp
// read_motor_position() 中，约第 70 行
double pulses_per_mm = 4000.0;  // 改为您的值

// write_motor_target_position() 中，约第 134 行
double pulses_per_mm = 4000.0;  // 改为您的值
```

### 改变位置范围限制

**在 `lifter_config.yaml` 中：**
```yaml
height_limits:
  min: 0      # 最小位置 (mm)
  max: 2000   # 最大位置 (mm)
```

### 改变速度和加速度

**在应用代码中：**
```cpp
lifter.set_motor_velocity(200);      // 改为您需要的 RPM
lifter.set_motor_acceleration(500);  // 改为您需要的加速度
```

---

## 🔧 调试技巧

### 启用详细日志
在 ROS2 launch 中设置日志级别：
```bash
ros2 launch lifter_ecat lifter_launch.py --log-level DEBUG
```

### 检查 SOEM 初始化
```cpp
if (!soem_initialized_) {
    RCLCPP_ERROR(logger, "SOEM not initialized!");
    return;
}
```

### 检查 PDO 缓冲区
```cpp
if (slavelist[1].inputs == NULL) {
    RCLCPP_ERROR(logger, "TxPDO input buffer not configured");
}
if (slavelist[1].outputs == NULL) {
    RCLCPP_ERROR(logger, "RxPDO output buffer not configured");
}
```

### 验证从站连接
```cpp
// 查看从站列表
lifter.dump_slave_info();

// 查看 PDO 映射信息
lifter.dump_pdo_mapping();
```

---

## 📋 典型代码框架

```cpp
#include <rclcpp/rclcpp.hpp>
#include "lifter_ecat/lifter.hpp"
#include "lifter_ecat/cia402.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motor_control");
    
    Lifter lifter(node);
    lifter.load_config();
    lifter.initialize_soem();
    
    // 【初始化】
    lifter.set_motor_control_word(0x000F);
    lifter.set_motor_mode(cia402::MODE_PROFILED_POSITION);
    lifter.set_motor_velocity(50);
    lifter.set_motor_acceleration(200);
    
    // 【运行】
    lifter.write_motor_target_position(500.0);
    
    // 【监测】
    for (int i = 0; i < 100; ++i) {
        auto status = lifter.read_motor_status_word();
        auto pos = lifter.read_motor_position();
        auto vel = lifter.read_motor_velocity();
        
        if (status.is_fault()) break;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // 【清理】
    lifter.set_motor_control_word(0x0000);
    lifter.shutdown();
    
    rclcpp::shutdown();
    return 0;
}
```

---

**更多信息**：见 `docs/CIA402_CONTROL_GUIDE.md` 完整 API 文档
