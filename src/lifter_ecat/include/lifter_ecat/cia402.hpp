#ifndef CIA402_HPP
#define CIA402_HPP

#include <cstdint>
#include <cstring>

/**
 * @file cia402.hpp
 * @brief CiA402 标准协议的数据结构和常量定义
 * 
 * CiA402 是 IEC 61800-7-1 标准，定义了电机驱动控制的 EtherCAT 通信协议。
 * 包括对象字典定义、状态机、控制字、状态字等。
 */

namespace cia402 {

// ═══════════════════════════════════════════════════════════════════════════
// 【Control Word 0x6040】 - 电机控制指令
// ═══════════════════════════════════════════════════════════════════════════
// 
// 位结构：
//   bit 0   = Shutdown (关闭)        0 -> 禁用   1 -> 使能
//   bit 1   = Enable operation       0 -> 禁用   1 -> 使能操作
//   bit 2   = Quick stop            0 -> 快速停止  1 -> 允许运行
//   bit 3   = Enable voltage        0 -> 禁用电压  1 -> 使能电压
//   bit 4   = Reserved
//   bit 5   = Reserved
//   bit 6   = Fault reset           短脉冲 = 复位故障
//   bit 7   = Reserved
//   bit 8-15 = Reserved / 特定于设备

struct ControlWord {
    uint16_t value = 0;
    
    // 便利方法
    void set_shutdown(bool enable) {
        if (enable) value |= (1 << 0);
        else value &= ~(1 << 0);
    }
    void set_enable_operation(bool enable) {
        if (enable) value |= (1 << 1);
        else value &= ~(1 << 1);
    }
    void set_quick_stop(bool enable) {
        if (enable) value |= (1 << 2);
        else value &= ~(1 << 2);
    }
    void set_enable_voltage(bool enable) {
        if (enable) value |= (1 << 3);
        else value &= ~(1 << 3);
    }
    void reset_fault() {
        value |= (1 << 6);
    }
    
    // 常用的控制命令
    void enable() {
        // 使能电机的标准流程：shutdown=1, enable=1, quick_stop=1, voltage=1
        value = 0x000F;  // 0000 1111
    }
    void disable() {
        value = 0x0000;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// 【Status Word 0x6041】 - 电机状态字
// ═══════════════════════════════════════════════════════════════════════════
//
// 位结构：
//   bit 0   = Ready to switch on
//   bit 1   = Switched on
//   bit 2   = Operation enabled
//   bit 3   = Fault
//   bit 4   = Voltage enabled
//   bit 5   = Quick stop
//   bit 6   = Switch on disabled
//   bit 7   = Warning
//   bit 8-15 = 特定于设备的状态

struct StatusWord {
    uint16_t value = 0;
    
    bool is_ready_to_switch_on() const { return (value & (1 << 0)) != 0; }
    bool is_switched_on() const { return (value & (1 << 1)) != 0; }
    bool is_operation_enabled() const { return (value & (1 << 2)) != 0; }
    bool is_fault() const { return (value & (1 << 3)) != 0; }
    bool is_voltage_enabled() const { return (value & (1 << 4)) != 0; }
    bool is_quick_stop_active() const { return (value & (1 << 5)) != 0; }
    bool is_switch_on_disabled() const { return (value & (1 << 6)) != 0; }
    bool has_warning() const { return (value & (1 << 7)) != 0; }
    
    // 返回状态机的当前阶段（CiA402 DSM）
    const char* get_state_name() const {
        // 常见 CiA402 状态编码（注意：不同厂商可能在高位做扩展）
        switch (value) {
            case 0x0000: return "Not Ready to Switch On";
            case 0x0040: return "Switch on Disabled";
            case 0x0021: return "Ready to Switch On";
            case 0x0023: return "Switched On";
            case 0x0027: return "Operation Enabled";
            case 0x0007: return "Quick Stop Active";
            case 0x000F: return "Fault Reaction Active";
            case 0x0008: return "Fault";
            // 部分驱动在高位附加扩展标志，比如 0x0270 此类值
            case 0x0270: return "Switch on Disabled (vendor extended)";
            default:     return "Unknown State";
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// 【Modes of Operation 0x6060】 - 工作模式 (Out)
// ═══════════════════════════════════════════════════════════════════════════
enum ModeOfOperation : int8_t {
    MODE_IDLE = 0,              // 空闲
    MODE_PROFILED_POSITION = 1, // 轮廓位置模式（有速度和加速度规划）
    MODE_VELOCITY = 3,          // 速度模式
    MODE_PROFILED_TORQUE = 4,   // 轮廓扭矩模式
    MODE_HOMING = 6,            // 回原点模式
    MODE_INTERPOLATED_POSITION = 7, // 插值位置模式
    MODE_CYCLIC_SYNCHRONOUS_POSITION = 8,   // 周期同步位置模式
    MODE_CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // 周期同步速度模式
    MODE_CYCLIC_SYNCHRONOUS_TORQUE = 10,    // 周期同步扭矩模式
};

// ═══════════════════════════════════════════════════════════════════════════
// 【Modes of Operation Display 0x6061】 - 当前工作模式 (In)
// ═══════════════════════════════════════════════════════════════════════════
inline const char* mode_of_operation_to_string(int8_t mode) {
    switch (mode) {
        case 0: return "Idle";
        case 1: return "Profiled Position";
        case 3: return "Velocity";
        case 4: return "Profiled Torque";
        case 6: return "Homing";
        case 7: return "Interpolated Position";
        case 8: return "Cyclic Synchronous Position";
        case 9: return "Cyclic Synchronous Velocity";
        case 10: return "Cyclic Synchronous Torque";
        default: return "Unknown";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 【PDO 输入/输出缓冲区结构】
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @struct RxPDO (Receive from master / Output)
 * 主站发送到从站的数据（电机驱动接收的命令）
 * 
 * 顺序（根据 0x1600 映射）：
 *   0x6040  16bit  Control Word
 *   0x607A  32bit  Target Position
 *   0x6081  32bit  Profile Velocity
 *   0x6060   8bit  Modes of Operation
 *   0x6071  16bit  Target Torque
 *   0x60FF  32bit  Target Velocity
 *   0x60B2  32bit  Acceleration
 */
struct RxPDO {
    uint16_t control_word;           // @ offset 0    0x6040
    int32_t target_position;         // @ offset 2    0x607A
    uint32_t profile_velocity;       // @ offset 6    0x6081
    int8_t modes_of_operation;       // @ offset 10   0x6060
    int16_t target_torque;           // @ offset 11   0x6071
    int32_t target_velocity;         // @ offset 13   0x60FF
    int32_t acceleration;            // @ offset 17   0x60B2
    
    // 总长度：21 字节
    static const int SIZE = 21;
} __attribute__((packed));

/**
 * @struct TxPDO (Transmit to master / Input)
 * 从站发送到主站的数据（电机驱动反馈的状态）
 * 
 * 顺序（根据 0x1A00 映射）：
 *   0x6041  16bit  Status Word
 *   0x6064  32bit  Position Actual Value
 *   0x606C  32bit  Velocity Actual Value
 *   0x6061   8bit  Modes of Operation Display
 *   0x603F  16bit  Error Code
 *   0x6077  16bit  Torque Actual Value
 *   0x60FD  32bit  Digital Inputs
 */
struct TxPDO {
    uint16_t status_word;            // @ offset 0    0x6041
    int32_t position_actual;         // @ offset 2    0x6064
    int32_t velocity_actual;         // @ offset 6    0x606C
    int8_t modes_of_operation_display; // @ offset 10 0x6061
    uint16_t error_code;             // @ offset 11   0x603F
    int16_t torque_actual;           // @ offset 13   0x6077
    uint32_t digital_inputs;         // @ offset 15   0x60FD
    
    // 总长度：19 字节
    static const int SIZE = 19;
} __attribute__((packed));

// ═══════════════════════════════════════════════════════════════════════════
// 【公共常量】
// ═══════════════════════════════════════════════════════════════════════════

// 对象字典 (Object Dictionary) 地址
constexpr uint16_t OD_CONTROL_WORD = 0x6040;
constexpr uint16_t OD_STATUS_WORD = 0x6041;
constexpr uint16_t OD_MODES_OF_OPERATION = 0x6060;
constexpr uint16_t OD_MODES_OF_OPERATION_DISPLAY = 0x6061;
constexpr uint16_t OD_ERROR_CODE = 0x603F;
constexpr uint16_t OD_TARGET_POSITION = 0x607A;
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t OD_ACCELERATION = 0x60B2;
constexpr uint16_t OD_TARGET_TORQUE = 0x6071;
constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;
constexpr uint16_t OD_POSITION_ACTUAL = 0x6064;
constexpr uint16_t OD_VELOCITY_ACTUAL = 0x606C;
constexpr uint16_t OD_TORQUE_ACTUAL = 0x6077;
constexpr uint16_t OD_DIGITAL_INPUTS = 0x60FD;

// 常见的错误码
constexpr uint16_t ERR_GENERIC = 0x1000;          // 通用错误
constexpr uint16_t ERR_CURRENT = 0x2100;          // 过流
constexpr uint16_t ERR_VOLTAGE = 0x2200;          // 电压异常
constexpr uint16_t ERR_TEMPERATURE = 0x2300;      // 温度异常
constexpr uint16_t ERR_MOTOR = 0x2500;            // 电机错误
constexpr uint16_t ERR_COMMUNICATION = 0x8110;    // 通信故障

} // namespace cia402

#endif // CIA402_HPP
