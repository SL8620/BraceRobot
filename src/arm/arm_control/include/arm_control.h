#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <vector>
#include <memory>
#include <cmath>

#include "kvaser.h"

namespace kvaser_motor_control
{

enum class ControlMode
{
    IDLE,
    SPEED,
    POSITION
};

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    static constexpr int NumOfMotors = 2;
    std::vector<std::vector<double>> preset_positions_;

    /* 电机 / CAN */
    MOTOR motors_[NumOfMotors];
    std::shared_ptr<KvaserForGold> canBus_;
    int can_channel_{0};

    /* 控制状态 */
    ControlMode mode_ = ControlMode::IDLE;
    bool position_enable=false;
    bool homed_{false};

    bool speed_enable_ = false;
    double target_speed_ = 0.0;
    const double speed_step_ = 0.1;

    std::vector<double> target_pos_{0.0};

    /* ROS */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pos_sub_;
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
    void stopAllMotors();
    void emergencyStop();
    void goHome();
};

} // namespace kvaser_motor_control
