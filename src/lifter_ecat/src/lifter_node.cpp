#include <rclcpp/rclcpp.hpp>
#include "lifter_ecat/lifter.hpp"
#include "lifter_ecat/cia402.hpp"
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lifter_node");

    Lifter lifter(node);

    // 加载配置文件
    lifter.load_config();

    // 初始化SOEM
    lifter.initialize_soem();

    // 打印从站信息与 PDO 映射（如果需要可以在 launch 中禁用）
    lifter.dump_slave_info();
    lifter.dump_pdo_mapping();

    // 演示从站状态管理：读取当前状态
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=== Demonstrating slave state management ===");
    lifter.read_slave_state(1);

    // ═══════════════════════════════════════════════════════════════
    // 简化版：只做电机使能 + 持续读取位置/状态，不发送运动指令
    // ═══════════════════════════════════════════════════════════════
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=== Enabling motor and reading position only ===");

    // 【1】按照 CiA402 状态机完整使能电机
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling motor via CiA402 state machine...");
    if (!lifter.enable_motor_cia402(1, 2000)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable motor via CiA402 sequence");
    }

    // 【2】读取一次初始位置/状态
    double initial_pos = lifter.read_motor_position();
    cia402::StatusWord status = lifter.read_motor_status_word();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Initial motor position (raw scaled): %.2f, status=0x%04x (%s)",
                initial_pos, status.value, status.get_state_name());

    // 【3】仅监测位置/速度/状态，不下发任何目标位置
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=== Monitoring motor feedback only (2 seconds) ===");
    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        double current_pos = lifter.read_motor_position();
        int32_t velocity = lifter.read_motor_velocity();
        int16_t torque = lifter.read_motor_torque();
        cia402::StatusWord current_status = lifter.read_motor_status_word();
        uint16_t error_code = lifter.read_motor_error_code();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "[%d/20] Pos=%.2f, Vel=%d, Torque=%d, Status=0x%04x (%s), Error=0x%04x",
                    i + 1, current_pos, velocity, torque,
                    current_status.value, current_status.get_state_name(), error_code);

        if (current_status.is_fault()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Motor fault detected! Error code: 0x%04x", error_code);
            break;
        }
    }

    // 【4】禁用电机
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disabling motor...");
    lifter.set_motor_control_word(0x0000);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Final motor position: %.2f",
                lifter.read_motor_position());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
