#include "arm_control.h"

using namespace std::chrono_literals;

namespace kvaser_motor_control
{

MotorControlNode::MotorControlNode()
: Node("motor_control_node")
{
    can_channel_ = this->declare_parameter<int>("can_channel", 0);
    RCLCPP_INFO(get_logger(), "Motor Control Node Started (CAN channel %d)", can_channel_);

    initMotors();
    initCAN();
    preset_positions_ = {
    { -15.0 * M_PI / 180.0,-15.0 * M_PI / 180.0,0 * M_PI / 180.0,0 * M_PI / 180.0},
    { -10.0 * M_PI / 180.0,-10.0 * M_PI / 180.0,20.0 * M_PI / 180.0,20.0 * M_PI / 180.0},
    { 0 * M_PI / 180.0,0 * M_PI / 180.0,0 * M_PI / 180.0,0 * M_PI / 180.0} 
    };
    goHome(); 
    initROS();

    timer_ = create_wall_timer(
        20ms, std::bind(&MotorControlNode::controlLoop, this));
}

MotorControlNode::~MotorControlNode()
{
    stopAllMotors();
    canBus_->canRelease();
}

/* ================== 初始化 ================== */
void MotorControlNode::initMotors()
{
    for (int i = 0; i < NumOfMotors; i++)
    {
        motors_[i].id = i + 1;
        motors_[i].connect = true;
        motors_[i].Kt_inv = 0.1;
        motors_[i].In = 10.0;
        motors_[i].Wn = 50.0;
        motors_[i].direction = 1;
        motors_[i].encoder.count = 131072;
        motors_[i].encoder.AbsZeroPos = 0;
        motors_[i].InitPos = 0.0;
    }
}

void MotorControlNode::initCAN()
{
    canBus_ = std::make_shared<KvaserForGold>(can_channel_, NumOfMotors, motors_, "MotorControl");

    for (int i = 0; i < NumOfMotors; i++)
    {
        canBus_->connectMotor(&motors_[i]);
        canBus_->RPDOconfig(&motors_[i], KvaserForGold::POSITION_MODE);
        canBus_->TPDOconfigPXVX(&motors_[i], 2);
    }

    RCLCPP_INFO(get_logger(), "CAN & Motors Initialized");
}

void MotorControlNode::initROS()
{
    pos_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor/position_cmd", 10,
        std::bind(&MotorControlNode::positionCallback, this, std::placeholders::_1));
}

/* ================== 键盘 ================== */
bool MotorControlNode::keyPressed()
{
    fd_set set;
    struct timeval tv{0, 0};
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    return select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0;
}

char MotorControlNode::readKey()
{
    char c;
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    read(STDIN_FILENO, &c, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

/* ================== 回调 ================== */
void MotorControlNode::positionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 至少需要为每个电机提供一个目标位置
    if (msg->position.size() < NumOfMotors)
    {
        RCLCPP_WARN(get_logger(), "Position command size (%zu) < NumOfMotors (%d)",
                    msg->position.size(), NumOfMotors);
        return;
    }

    // 收到位置指令时自动切换到位置模式
    if (mode_ != ControlMode::POSITION)
    {
        switchToPositionMode();
    }

    for (int i = 0; i < NumOfMotors; i++)
    {
        target_pos_[i] = msg->position[i];
    }

    // 立即发送一次位置指令
    sendPositionCommand();
}

/* ================== 主循环 ================== */
void MotorControlNode::controlLoop()
{
    if (!homed_)
    {
        goHome();
        homed_ = true;
        return;
    }

    if (keyPressed())
        handleKey(readKey());

    if (mode_ == ControlMode::SPEED)
        sendSpeedCommand();
    else if (mode_ == ControlMode::POSITION)
        sendPositionCommand();
}


/* ================== 键盘逻辑 ================== */

void MotorControlNode::handleKey(char key)
{
    switch (key)
    {
    case '1': switchToSpeedMode(); break;
    case '3': switchToPositionMode(); break;

    case '4':if (mode_ == ControlMode::POSITION)
    {
        target_pos_ = preset_positions_[0];
        position_enable = true;
    }
    break;

    case '5':
    if (mode_ == ControlMode::POSITION)
    {
        target_pos_ = preset_positions_[1];
        position_enable = true;
    }
    break;

    case '6':
    if (mode_ == ControlMode::POSITION)
    {
        target_pos_ = preset_positions_[2];
        position_enable = true;
    }
    break;


    case ' ': if (mode_ == ControlMode::SPEED) speed_enable_ = !speed_enable_; break;
    case '8': if (mode_ == ControlMode::SPEED) target_speed_ += speed_step_; break;
    case '2': if (mode_ == ControlMode::SPEED) target_speed_ -= speed_step_; break;

    case '0': emergencyStop(); break;
    default: break;
    }
}

/* ================== 模式切换 ================== */
void MotorControlNode::switchToSpeedMode()
{
    stopAllMotors();
    mode_ = ControlMode::SPEED;
    speed_enable_ = false;
    target_speed_ = 0.0;
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->modeChoose(&motors_[i], KvaserForGold::SPEED_MODE);
    RCLCPP_INFO(get_logger(), "Switched to SPEED mode");
}

void MotorControlNode::switchToPositionMode()
{
    stopAllMotors();
    mode_ = ControlMode::POSITION;
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->modeChoose(&motors_[i], KvaserForGold::POSITION_MODE);
    RCLCPP_INFO(get_logger(), "Switched to POSITION mode");
}
/* ================= 预设位置 ================= */

void MotorControlNode::moveToPreset(size_t index)
{
    if (index >= preset_positions_.size())
        return;

    if (preset_positions_[index].size() != NumOfMotors)
    {
        RCLCPP_ERROR(get_logger(), "Preset position size mismatch");
        return;
    }

    stopAllMotors();

    mode_ = ControlMode::POSITION;

    // 切换位置模式
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->modeChoose(&motors_[i], KvaserForGold::POSITION_MODE);

    // ⭐ 核心：整体赋值（不是用 i）
    target_pos_ = preset_positions_[index];

    RCLCPP_INFO(
        get_logger(),
        "Move to preset %ld: [%.3f, %.3f]",
        index + 4,
        target_pos_[0],target_pos_[1]);

    // 连续发送，保证 CAN 接收
    for (int k = 0; k < 50; k++)
    {
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->PositionMode(&motors_[i], 0.0, 5); // 20% 速度

        std::this_thread::sleep_for(20ms);
    }
}


/* ================== CAN 指令 ================== */
void MotorControlNode::sendSpeedCommand()
{
    double v = speed_enable_ ? target_speed_ : 0.0;
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->SpeedMode(&motors_[i], v);
}

void MotorControlNode::sendPositionCommand()
{
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->PositionMode(&motors_[i], target_pos_[i], 5);
}

void MotorControlNode::stopAllMotors()
{
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->SpeedMode(&motors_[i], 0.0);
}

void MotorControlNode::emergencyStop()
{
    stopAllMotors();
    mode_ = ControlMode::IDLE;
    RCLCPP_WARN(get_logger(), "!!! EMERGENCY STOP !!!");
}

void MotorControlNode::goHome()
{
    RCLCPP_INFO(get_logger(), "Going to home position (0 rad)");

    // 1. 切换位置模式
    mode_ = ControlMode::POSITION;
    for (int i = 0; i < NumOfMotors; i++)
        canBus_->modeChoose(&motors_[i], KvaserForGold::POSITION_MODE);

    // 2. 设置目标为 0
    for (int i = 0; i < NumOfMotors; i++)
        target_pos_[i] = 0.0;

    // 3. 多次发送位置指令（保证收到）
    for (int k = 0; k < 50; k++)
    {
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->PositionMode(&motors_[i], 0.0, 5); // 20% 速度

        std::this_thread::sleep_for(20ms);
    }

    // 4. 停止
    stopAllMotors();
    mode_ = ControlMode::IDLE;

    RCLCPP_INFO(get_logger(), "Home position reached");
}


} // namespace kvaser_motor_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvaser_motor_control::MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
