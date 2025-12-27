# CiA402 标准控制函数使用指南

## 概述

本文档说明如何使用 `Lifter` 类提供的 CiA402 标准控制函数。CiA402 是 IEC 61800-7-1 标准，定义了 EtherCAT 上的驱动器通信协议。

## PDO 映射结构

### 接收 PDO (RxPDO 0x1600) - 主站 → 从站（命令）

```
字节偏移    对象代码    名称                  类型        说明
──────────────────────────────────────────────────────────────
0 (2B)     0x6040     Control Word         uint16      电机控制字
2 (4B)     0x607A     Target Position      int32       目标位置（脉冲）
6 (4B)     0x6081     Profile Velocity     uint32      轮廓速度（RPM）
10 (1B)    0x6060     Modes of Operation   int8        工作模式
11 (2B)    0x6071     Target Torque        int16       目标力矩（0.1% FT）
13 (4B)    0x60FF     Target Velocity      int32       目标速度
17 (4B)    0x60B2     Acceleration         uint32      加速度
─────────────────────────────────────────────────────────────
总大小：21 字节
```

### 发送 PDO (TxPDO 0x1A00) - 从站 → 主站（反馈）

```
字节偏移    对象代码    名称                    类型        说明
──────────────────────────────────────────────────────────────
0 (2B)     0x6041     Status Word           uint16      电机状态字
2 (4B)     0x6064     Position Actual Value int32       实际位置（脉冲）
6 (4B)     0x606C     Velocity Actual Value int32       实际速度
10 (1B)    0x6061     Modes of Operation Dis int8       当前工作模式
11 (2B)    0x603F     Error Code            uint16      错误码
13 (2B)    0x6077     Torque Actual Value   int16       实际力矩
15 (4B)    0x60FD     Digital Inputs        uint32      数字输入
─────────────────────────────────────────────────────────────
总大小：19 字节
```

## 控制字 (Control Word 0x6040) 详解

控制字用于向电机驱动器发送命令。每一位有特定含义：

```
位  名称                 0 = 禁用         1 = 使能
────────────────────────────────────────────────────
0   Shutdown            禁用电源         准备就绪
1   Enable Operation    禁用操作         使能操作
2   Quick Stop          快速停止         允许运行
3   Enable Voltage      禁用电压         使能电压
4-5 保留
6   Fault Reset         无效             复位故障（脉冲）
7-15 保留或设备特定
```

### 常用控制字值

| 控制字    | 十进制 | 含义 |
|---------|------|------|
| 0x0000  | 0    | 完全禁用 |
| 0x000F  | 15   | 完全使能（推荐） |
| 0x0007  | 7    | 快速停止（清除位 2） |

## 工作模式 (Mode of Operation)

电机支持多种工作模式。最常用的是轮廓位置模式（MODE 1）：

| 模式值  | 名称 | 说明 |
|--------|------|------|
| 0 | Idle | 空闲模式 |
| **1** | **Profiled Position** | **轮廓位置模式**（推荐用于电缸位置控制）|
| 3 | Velocity | 速度模式 |
| 4 | Profiled Torque | 轮廓扭矩模式 |
| 6 | Homing | 回原点模式 |
| 7 | Interpolated Position | 插值位置模式 |
| 8 | Cyclic Synchronous Position | 周期同步位置模式 |
| 9 | Cyclic Synchronous Velocity | 周期同步速度模式 |
| 10 | Cyclic Synchronous Torque | 周期同步扭矩模式 |

## API 函数参考

### 位置控制

#### `double read_motor_position()`
读取电机的实际位置。

**返回值**：位置（毫米）

**说明**：
- 从 TxPDO 的 0x6064 (Position Actual Value) 读取原始脉冲值
- 使用配置的转换因子（1000 脉冲/mm）转换为物理单位
- 应用位置偏移 (`motor_position_offset_`)
- 如果 PDO 未配置，返回缓存的上次读取值

**示例**：
```cpp
double pos = lifter.read_motor_position();
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position: %.2f mm", pos);
```

#### `bool write_motor_target_position(double position)`
设置电机的目标位置。

**参数**：
- `position`：目标位置（毫米），必须在 height_limits 范围内www

**返回值**：成功 (true) 或失败 (false)

**说明**：
- 检查位置范围
- 应用位置偏移，转换为脉冲值
- 写入 RxPDO 的 0x607A (Target Position)
- 同时设置控制字和工作模式为轮廓位置模式

**示例**：
```cpp
if (lifter.write_motor_target_position(500.0)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Target position set to 500 mm");
} else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set position");
}
```

#### `void set_motor_position_offset(double offset)`
设置位置偏移值（用于原点校准）。

**参数**：
- `offset`：偏移量（毫米）

**说明**：
- 如果电机的机械原点不在物理零点，可通过此方法设置偏移
- 所有位置读写操作都会自动应用此偏移

**示例**：
```cpp
// 假设电机原点实际位置在 50mm，设置偏移以使其显示为 0mm
lifter.set_motor_position_offset(50.0);
```

#### `double get_motor_position_offset() const`
获取当前的位置偏移值。

**返回值**：偏移量（毫米）

### 速度控制

#### `bool set_motor_velocity(int32_t velocity_rpm)`
设置轮廓位置模式下的运动速度。

**参数**：
- `velocity_rpm`：速度值（单位根据驱动器配置，通常为 RPM 或 mm/s）

**返回值**：成功 (true) 或失败 (false)

**说明**：
- 仅在轮廓位置模式下有效
- 写入 RxPDO 的 0x6081 (Profile Velocity)

**示例**：
```cpp
// 设置速度为 100 RPM
if (lifter.set_motor_velocity(100)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity set");
}
```

#### `int32_t read_motor_velocity()`
读取电机的实际速度。

**返回值**：实际速度

**说明**：
- 从 TxPDO 的 0x606C (Velocity Actual Value) 读取
- 反映电机当前的实时速度

### 加速度控制

#### `bool set_motor_acceleration(uint32_t acceleration)`
设置轮廓位置模式下的加速度。

**参数**：
- `acceleration`：加速度值（单位根据驱动器，通常为 rpm/s 或 mm/s²）

**返回值**：成功 (true) 或失败 (false)

**说明**：
- 控制位置运动的加速过程
- 写入 RxPDO 的 0x60B2 (Acceleration)

**示例**：
```cpp
// 设置加速度为 500
lifter.set_motor_acceleration(500);
```

### 控制字和模式

#### `bool set_motor_control_word(uint16_t control_word)`
直接设置电机控制字。

**参数**：
- `control_word`：16 位控制字值

**返回值**：成功 (true) 或失败 (false)

**说明**：
- 通常使用 0x000F 表示完全使能
- 位 6 用于故障复位（需要脉冲）
- 此函数直接修改 RxPDO 的控制字

**示例**：
```cpp
// 使能电机
lifter.set_motor_control_word(0x000F);

// 快速停止
lifter.set_motor_control_word(0x0007);
```

#### `bool set_motor_mode(cia402::ModeOfOperation mode)`
设置电机的工作模式。

**参数**：
- `mode`：来自 `cia402::ModeOfOperation` 枚举的模式

**返回值**：成功 (true) 或失败 (false)

**说明**：
- 推荐使用 `cia402::MODE_PROFILED_POSITION` (值 1) 进行电缸位置控制
- 写入 RxPDO 的 0x6060 (Modes of Operation)

**示例**：
```cpp
// 设置为轮廓位置模式
lifter.set_motor_mode(cia402::MODE_PROFILED_POSITION);

// 或设置为速度模式
lifter.set_motor_mode(cia402::MODE_VELOCITY);
```

### 状态读取

#### `cia402::StatusWord read_motor_status_word()`
读取电机的状态字。

**返回值**：`cia402::StatusWord` 结构体

**说明**：
- 包含电机的当前状态和各种标志位
- StatusWord 结构体提供便利的查询方法

**StatusWord 方法**：
- `bool is_operation_enabled()`：电机是否处于使能运行状态
- `bool is_fault()`：是否有故障
- `bool is_voltage_enabled()`：电压是否使能
- `const char* get_state_name()`：返回当前状态的名称字符串

**示例**：
```cpp
cia402::StatusWord status = lifter.read_motor_status_word();

if (status.is_operation_enabled()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor is running");
}

if (status.is_fault()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor fault detected!");
}

RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current state: %s", status.get_state_name());
```

#### `int16_t read_motor_torque()`
读取电机的实际力矩。

**返回值**：实际力矩（单位：0.1% FT，基数100 = 100%额定转矩）

**说明**：
- 从 TxPDO 的 0x6077 (Torque Actual Value) 读取
- 用于监测电机的负载情况

#### `uint16_t read_motor_error_code()`
读取电机的错误码。

**返回值**：错误码（0 = 无错误）

**常见错误码**：
- `0x0000`：无错误
- `0x1000`：通用错误
- `0x2100`：过流
- `0x2200`：电压异常
- `0x2300`：温度异常
- `0x2500`：电机错误
- `0x8110`：通信故障

**说明**：
- 从 TxPDO 的 0x603F (Error Code) 读取
- 如果错误码非零，会自动输出 WARN 日志

**示例**：
```cpp
uint16_t error = lifter.read_motor_error_code();
if (error != 0) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
               "Motor error: 0x%04x", error);
}
```

## 典型的位置控制流程

```cpp
void position_control_example() {
    Lifter lifter(node);
    lifter.load_config();
    lifter.initialize_soem();
    
    // 1. 使能电机
    lifter.set_motor_control_word(0x000F);
    
    // 2. 设置工作模式为轮廓位置模式
    lifter.set_motor_mode(cia402::MODE_PROFILED_POSITION);
    
    // 3. 设置速度和加速度
    lifter.set_motor_velocity(100);      // 100 RPM
    lifter.set_motor_acceleration(500);  // 500 rpm/s
    
    // 4. 读取当前位置
    double current_pos = lifter.read_motor_position();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current pos: %.2f mm", current_pos);
    
    // 5. 设置目标位置
    double target_pos = 500.0;  // 500 mm
    if (lifter.write_motor_target_position(target_pos)) {
        // EtherCAT 周期性发送此值，驱动器会执行位置控制
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving to %.2f mm", target_pos);
    }
    
    // 6. 监测状态
    while (rclcpp::ok()) {
        cia402::StatusWord status = lifter.read_motor_status_word();
        int32_t velocity = lifter.read_motor_velocity();
        int16_t torque = lifter.read_motor_torque();
        
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), 
                    "Status: %s, Velocity: %d, Torque: %d",
                    status.get_state_name(), velocity, torque);
        
        // 检查故障
        if (status.is_fault()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor fault!");
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 7. 禁用电机
    lifter.set_motor_control_word(0x0000);
}
```

## 数据类型定义

### `cia402::RxPDO` (输出 PDO)
```cpp
struct RxPDO {
    uint16_t control_word;           // @ 0
    int32_t target_position;         // @ 2
    uint32_t profile_velocity;       // @ 6
    int8_t modes_of_operation;       // @ 10
    int16_t target_torque;           // @ 11
    int32_t target_velocity;         // @ 13
    int32_t acceleration;            // @ 17
    // 总计 21 字节
};
```

### `cia402::TxPDO` (输入 PDO)
```cpp
struct TxPDO {
    uint16_t status_word;            // @ 0
    int32_t position_actual;         // @ 2
    int32_t velocity_actual;         // @ 6
    int8_t modes_of_operation_display; // @ 10
    uint16_t error_code;             // @ 11
    int16_t torque_actual;           // @ 13
    uint32_t digital_inputs;         // @ 15
    // 总计 19 字节
};
```

### `cia402::StatusWord` 
提供对状态字的便利访问：
```cpp
struct StatusWord {
    uint16_t value;
    
    bool is_ready_to_switch_on() const;
    bool is_switched_on() const;
    bool is_operation_enabled() const;
    bool is_fault() const;
    bool is_voltage_enabled() const;
    bool is_quick_stop_active() const;
    bool is_switch_on_disabled() const;
    bool has_warning() const;
    const char* get_state_name() const;
};
```

## 注意事项

1. **脉冲转换因子**：代码中使用 `1000 脉冲/mm` 的转换因子。如果您的驱动器或编码器不同，请修改 `read_motor_position()` 和 `write_motor_target_position()` 中的 `pulses_per_mm` 变量。

2. **PDO 更新频率**：EtherCAT 通信通常在 1ms 或更短的周期内进行。位置更新不是即时的，而是在下一个 PDO 循环时生效。

3. **工作模式**：轮廓位置模式（MODE 1）自动处理加速/减速的速度规划。其他模式需要不同的控制策略。

4. **错误处理**：所有 set_* 函数都会检查 SOEM 初始化状态和 PDO 缓冲区配置。如果失败，会记录详细的错误日志。

5. **线程安全**：当前实现不是线程安全的。如果在多个线程中调用这些函数，需要添加互斥锁保护。

