#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <vector>
#include <memory>
#include <cmath>

#include <array>

#include "kvaser.h"

namespace kvaser_motor_control
{

enum class ControlMode
{
    IDLE,
    SPEED,
    POSITION,
    TORQUE
};

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    static constexpr int NumOfMotors = 6;
    std::vector<std::vector<double>> preset_positions_;

    /* 电机 / CAN */
    MOTOR motors_[NumOfMotors];
    std::shared_ptr<KvaserForGold> canBus_;
    int can_channel_{0};

    // 逻辑零点偏移：用于“将当前角度设为 0”功能
    std::array<double, NumOfMotors> zero_offsets_{};

    /* 控制状态 */
    ControlMode mode_ = ControlMode::IDLE;
    bool position_enable=false;
    // 禁用启动回零逻辑（避免电机6回0撞限位）
    bool homed_{true};

    bool speed_enable_ = false;
    double target_speed_ = 0.0;
    const double speed_step_ = 0.1;

    std::vector<double> target_pos_;
    std::vector<double> target_torque_;

    // 阻抗控制（外环）：qd/qd_dot -> tau
    bool impedance_active_{false};
    std::vector<double> impedance_qd_;
    std::vector<double> impedance_qd_dot_;
    double impedance_kp_{0.0};
    double impedance_kd_{0.0};

    // PP + 力矩保护（仅对 1/2 号电机）
    bool pp_torque_protect_enable_{false};
    int pp_max_torque_6072_{0};
    int pp_torque_threshold_6077_{0};
    int pp_trip_count_{3};
    std::array<int, NumOfMotors> pp_over_count_{};
    std::array<bool, NumOfMotors> pp_tripped_{};

    /* ROS */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr impedance_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pp_reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_zero_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* 初始化 */
    void initMotors();
    void initCAN();
    void initROS();

    /* 键盘 */
    bool keyPressed();
    char readKey();

    /* 回调 */
    void positionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void impedancePositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ppResetCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void setZeroCallback(const std_msgs::msg::Int32::SharedPtr msg);

    /* 主循环 */
    void controlLoop();

    /* 键盘逻辑 */
    void handleKey(char key);

    /* 模式切换 */
    void switchToSpeedMode();
    void switchToPositionMode();
    void moveToPreset(size_t index);
    /* CAN 指令 */
    void sendSpeedCommand();
    void sendPositionCommand();
    void sendTorqueCommand();
    void sendImpedanceCommand(double dt);
    void stopAllMotors();
    void emergencyStop();
    void goHome();
    void publishJointStates();
};

} // namespace kvaser_motor_control
