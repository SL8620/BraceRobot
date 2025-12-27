#ifndef LIFTER_HPP
#define LIFTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <soem/soem.h>
#include <yaml-cpp/yaml.h>
#include "cia402.hpp"
#include <fstream>
#include <string>

class Lifter
{
public:
    Lifter(rclcpp::Node::SharedPtr node);
    ~Lifter();
    void shutdown();
    void load_config();
    void initialize_soem();
    void cmd_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void timer_callback();
    void publish_status(const std::string& status);
    void publish_height(double height);
    // 新增：打印从站信息与 PDO 映射
    void dump_slave_info();
    void dump_pdo_mapping();
    // 新增：从站状态管理
    uint16 read_slave_state(uint16 slave);
    bool set_slave_state(uint16 slave, uint16 target_state);
    bool verify_slave_state(uint16 slave, uint16 expected_state);
    // 新增：电机位置读取
    double read_motor_position();
    bool write_motor_target_position(double position);
    double get_motor_position_offset() const { return motor_position_offset_; }
    void set_motor_position_offset(double offset) { motor_position_offset_ = offset; }

    // 新增：CiA402 控制
    bool set_motor_control_word(uint16_t control_word);
    cia402::StatusWord read_motor_status_word();
    bool set_motor_mode(cia402::ModeOfOperation mode);
    bool set_motor_velocity(int32_t velocity_rpm);
    bool set_motor_acceleration(uint32_t acceleration);
    int32_t read_motor_velocity();
    int16_t read_motor_torque();
    uint16_t read_motor_error_code();

    // 新增：按照 CiA402 状态机完整使能电机（Not Ready → Operation Enabled）
    bool enable_motor_cia402(uint16 slave = 1, int timeout_ms_per_step = 1000);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string ethernet_interface_;
    /* SOEM context for ecx_* API (SOEM v2+) */
    ecx_contextt soem_context_;
    bool soem_initialized_ = false;
    struct {
        int min;
        int max;
    } height_limits_;
    struct {
        int max;
    } speed_limits_;
    struct {
        int deceleration;
    } stop_params_;
    // 电机位置相关
    double motor_position_offset_ = 0.0;  // 位置偏移（mm）
    int32_t motor_actual_position_ = 0;   // 当前位置（原始值，通常是步数或计数）
    int32_t motor_target_position_ = 0;   // 目标位置（原始值）
    
    // CiA402 PDO 数据缓存（用于快速访问）
    cia402::RxPDO rx_pdo_;  // 输出 PDO（主站 -> 从站）
    cia402::TxPDO tx_pdo_;  // 输入 PDO（从站 -> 主站）
};

#endif  // LIFTER_HPP
