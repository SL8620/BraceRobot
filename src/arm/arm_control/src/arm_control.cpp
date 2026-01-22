#include "arm_control.h"

using namespace std::chrono_literals;

namespace kvaser_motor_control
{

MotorControlNode::MotorControlNode()
: Node("motor_control_node")
{
    can_channel_ = this->declare_parameter<int>("can_channel", 0);
    RCLCPP_INFO(get_logger(), "Motor Control Node Started (CAN channel %d)", can_channel_);

    // 初始化逻辑零点偏移为 0
    zero_offsets_.fill(0.0);

    initMotors();
    initCAN();
    // 为每个电机初始化目标位置向量
    target_pos_.assign(NumOfMotors, 0.0);
    // PT 模式下的目标力矩（默认 0）
    target_torque_.assign(NumOfMotors, 0.0);

    // 阻抗控制参数（默认 0：不输出力矩）
    impedance_kp_ = this->declare_parameter<double>("impedance_kp", 0.0);
    impedance_kd_ = this->declare_parameter<double>("impedance_kd", 0.0);
    impedance_qd_.assign(NumOfMotors, 0.0);
    impedance_qd_dot_.assign(NumOfMotors, 0.0);

    // PP + 力矩保护参数（默认关闭，避免误配置）
    pp_torque_protect_enable_ = this->declare_parameter<bool>("pp_torque_protect_enable", false);
    pp_max_torque_6072_ = this->declare_parameter<int>("pp_max_torque_6072", 0);
    pp_torque_threshold_6077_ = this->declare_parameter<int>("pp_torque_threshold_6077", 0);
    pp_trip_count_ = this->declare_parameter<int>("pp_trip_count", 3);
    preset_positions_ = {
    { -15.0 * M_PI / 180.0, -15.0 * M_PI / 180.0,  0.0,               0.0,               0.0, 0.0},
    { -10.0 * M_PI / 180.0, -10.0 * M_PI / 180.0, 20.0 * M_PI / 180.0,20.0 * M_PI / 180.0,0.0, 0.0},
    {  0.0,                  0.0,                  0.0,               0.0,               0.0, 0.0} 
    };
    // 启动时不执行回零/回默认位姿（避免 6 号电机回 0 撞限位）
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
        // 启动默认配置为 PP（Profile Position）模式
        canBus_->RPDOconfig(&motors_[i], KvaserForGold::POSITION_MODE);
        canBus_->modeChoose(&motors_[i], KvaserForGold::POSITION_MODE);
        canBus_->TPDOconfigPXVX(&motors_[i], 2);
    }

    // 仅对电机 1/2 设置最大力矩限制（0x6072），用于人机交互夹取保护
    if (pp_torque_protect_enable_ && pp_max_torque_6072_ > 0)
    {
        for (int i = 0; i < NumOfMotors; i++)
        {
            const int motor_id = motors_[i].id;
            if (motor_id == 1 || motor_id == 2)
            {
                const bool ok = canBus_->SDOWriteU16(&motors_[i], 0x6072, 0x00, (uint16_t)pp_max_torque_6072_);
                RCLCPP_INFO(get_logger(), "Set motor %d 0x6072 max_torque=%d (%s)", motor_id, pp_max_torque_6072_, ok ? "OK" : "FAIL");
            }
        }
    }

    RCLCPP_INFO(get_logger(), "CAN & Motors Initialized");
}

void MotorControlNode::initROS()
{
    pos_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor/position_cmd", 10,
        std::bind(&MotorControlNode::positionCallback, this, std::placeholders::_1));

    // PT 力矩指令：使用 JointState.effort[0..5]
    torque_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor/torque_cmd", 10,
        std::bind(&MotorControlNode::torqueCallback, this, std::placeholders::_1));

    // 阻抗期望位置：使用 JointState.position[0..5]（可选 velocity 作为 qd_dot）
    impedance_pos_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor/impedance_position_cmd", 10,
        std::bind(&MotorControlNode::impedancePositionCallback, this, std::placeholders::_1));

    // 力矩保护复位：data=0 复位全部，data=1/2 复位对应电机
    pp_reset_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/motor/pp_torque_protect_reset", 10,
        std::bind(&MotorControlNode::ppResetCallback, this, std::placeholders::_1));

    // 发布 6 个电机的实时关节状态（位置/速度）
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/motor/joint_states", 10);

    // 使能 / 失能 所有电机：/motor/enable_cmd, data=1 使能，0 失能
    enable_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/motor/enable_cmd", 10,
        std::bind(&MotorControlNode::enableCallback, this, std::placeholders::_1));

    // 设置某个电机当前物理位置为逻辑 0：/motor/set_zero, data=1-6
    set_zero_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/motor/set_zero", 10,
        std::bind(&MotorControlNode::setZeroCallback, this, std::placeholders::_1));
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

    // 如果当前在 TORQUE/阻抗控制中，位置话题不触发切回位置模式
    if (mode_ == ControlMode::TORQUE)
    {
        RCLCPP_WARN(get_logger(), "Ignore /motor/position_cmd while in TORQUE mode (impedance/PT)");
        return;
    }

    // 收到位置指令时自动切换到位置模式
    if (mode_ != ControlMode::POSITION)
        switchToPositionMode();

    for (int i = 0; i < NumOfMotors; i++)
    {
        // 逻辑指令 + 零点偏移 = 实际物理目标位置
        target_pos_[i] = msg->position[i] + zero_offsets_[i];
    }

    // 立即发送一次位置指令
    sendPositionCommand();
}

/* ================== 主循环 ================== */
void MotorControlNode::controlLoop()
{
    // homed_ 默认 true：按要求不执行启动回零

    if (keyPressed())
        handleKey(readKey());

    if (mode_ == ControlMode::SPEED)
        sendSpeedCommand();
    else if (mode_ == ControlMode::POSITION)
        sendPositionCommand();
    else if (mode_ == ControlMode::TORQUE)
    {
        if (impedance_active_)
            sendImpedanceCommand(0.02); // timer_ 20ms
        else
            sendTorqueCommand();
    }

    // 周期性发布电机的实际位置/速度
    publishJointStates();
}

void MotorControlNode::ppResetCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    const int id = msg->data;
    if (id == 0)
    {
        pp_tripped_.fill(false);
        pp_over_count_.fill(0);
        RCLCPP_INFO(get_logger(), "PP torque protect reset: all motors");
        return;
    }
    if (id >= 1 && id <= NumOfMotors)
    {
        pp_tripped_[id - 1] = false;
        pp_over_count_[id - 1] = 0;
        RCLCPP_INFO(get_logger(), "PP torque protect reset: motor %d", id);
        return;
    }
    RCLCPP_WARN(get_logger(), "PP torque protect reset: invalid id=%d", id);
}

void MotorControlNode::torqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->effort.size() < NumOfMotors)
    {
        RCLCPP_WARN(get_logger(), "Torque command size (%zu) < NumOfMotors (%d)",
                    msg->effort.size(), NumOfMotors);
        return;
    }

    if (mode_ != ControlMode::TORQUE)
    {
        stopAllMotors();
        mode_ = ControlMode::TORQUE;
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->modeChoose(&motors_[i], KvaserForGold::TORQUE_MODE);
        RCLCPP_INFO(get_logger(), "Switched to TORQUE mode");
    }

    for (int i = 0; i < NumOfMotors; i++)
        target_torque_[i] = msg->effort[i];

    // 显式力矩指令优先：收到后退出阻抗控制
    impedance_active_ = false;

    sendTorqueCommand();
}

void MotorControlNode::impedancePositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < NumOfMotors)
    {
        RCLCPP_WARN(get_logger(), "Impedance position cmd size (%zu) < NumOfMotors (%d)",
                    msg->position.size(), NumOfMotors);
        return;
    }

    if (mode_ != ControlMode::TORQUE)
    {
        stopAllMotors();
        mode_ = ControlMode::TORQUE;
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->modeChoose(&motors_[i], KvaserForGold::TORQUE_MODE);
        RCLCPP_INFO(get_logger(), "Switched to TORQUE mode (impedance)");
    }

    for (int i = 0; i < NumOfMotors; i++)
        impedance_qd_[i] = msg->position[i] + zero_offsets_[i];

    if (msg->velocity.size() >= NumOfMotors)
    {
        for (int i = 0; i < NumOfMotors; i++)
            impedance_qd_dot_[i] = msg->velocity[i];
    }
    else
    {
        for (int i = 0; i < NumOfMotors; i++)
            impedance_qd_dot_[i] = 0.0;
    }

    impedance_active_ = true;
}


/* ================== 键盘逻辑 ================== */

void MotorControlNode::handleKey(char key)
{
    switch (key)
    {
    case '1': switchToSpeedMode(); break;
    case '3': switchToPositionMode(); break;
    case '7':
        stopAllMotors();
        mode_ = ControlMode::TORQUE;
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->modeChoose(&motors_[i], KvaserForGold::TORQUE_MODE);
        RCLCPP_INFO(get_logger(), "Switched to TORQUE mode");
        break;

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

    // ⭐ 核心：整体赋值（不是用 i），并叠加零点偏移
    for (int i = 0; i < NumOfMotors; ++i)
    {
        target_pos_[i] = preset_positions_[index][i] + zero_offsets_[i];
    }

    RCLCPP_INFO(
        get_logger(),
        "Move to preset %ld: [%.3f, %.3f]",
        index + 4,
        target_pos_[0],target_pos_[1]);

    // 连续发送，保证 CAN 接收
    for (int k = 0; k < 50; k++)
    {
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->PositionMode(&motors_[i], 0.0, 2); // 20% 速度

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
    // 仅在 PP 位置模式下：使用 PDO 下发（RPDO: 6040 + 607A）
    for (int i = 0; i < NumOfMotors; i++)
    {
        const int motor_id = motors_[i].id;

        // 仅对 1/2 号电机进行“力矩触发停止并保持当前位置”
        if (pp_torque_protect_enable_ && (motor_id == 1 || motor_id == 2) && pp_torque_threshold_6077_ > 0)
        {
            if (!pp_tripped_[i])
            {
                int16_t tau_actual = 0;
                const bool ok = canBus_->SDOReadI16(&motors_[i], 0x6077, 0x00, tau_actual);
                if (ok && std::abs((int)tau_actual) >= pp_torque_threshold_6077_)
                    pp_over_count_[i]++;
                else
                    pp_over_count_[i] = 0;

                if (pp_over_count_[i] >= pp_trip_count_)
                {
                    // 触发后：锁定目标为当前位置（保持位置）
                    try
                    {
                        const double q_now = canBus_->GetPosition(&motors_[i]);
                        target_pos_[i] = q_now;
                    }
                    catch (...)
                    {
                        // 读不到就保持原目标，但标记已触发
                    }

                    pp_tripped_[i] = true;
                    RCLCPP_WARN(get_logger(), "Motor %d torque protect TRIPPED (0x6077=%d), hold position", motor_id, (int)tau_actual);
                }
            }
        }

        canBus_->SendPositionCommand(&motors_[i], target_pos_[i]);
    }
}

void MotorControlNode::sendTorqueCommand()
{
    for (int i = 0; i < NumOfMotors; i++)
    canBus_->SendTorqueCommand(&motors_[i], target_torque_[i]);
}

void MotorControlNode::sendImpedanceCommand(double dt)
{
    // tau = Kp*(qd - q) + Kd*(qd_dot - qdot)
    (void)dt;

    for (int i = 0; i < NumOfMotors; i++)
    {
        double q = 0.0;
        double qdot = 0.0;
        try
        {
            q = canBus_->GetPosition(&motors_[i]);
            qdot = canBus_->GetVelocity(&motors_[i]);
        }
        catch (...)
        {
            // 读取失败时不给力矩，避免失控
            canBus_->SendTorqueCommand(&motors_[i], 0.0);
            continue;
        }

        const double e = impedance_qd_[i] - q;
        const double edot = impedance_qd_dot_[i] - qdot;
        const double tau = impedance_kp_ * e + impedance_kd_ * edot;
        canBus_->SendTorqueCommand(&motors_[i], tau);
    }
}

void MotorControlNode::stopAllMotors()
{
    if (mode_ == ControlMode::TORQUE)
    {
        for (int i = 0; i < NumOfMotors; i++)
            canBus_->SendTorqueCommand(&motors_[i], 0.0);
        return;
    }

    for (int i = 0; i < NumOfMotors; i++)
        canBus_->SpeedMode(&motors_[i], 0.0);
}

void MotorControlNode::emergencyStop()
{
    stopAllMotors();
    mode_ = ControlMode::IDLE;
    RCLCPP_WARN(get_logger(), "!!! EMERGENCY STOP !!!");
}

void MotorControlNode::enableCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!canBus_)
    {
        RCLCPP_WARN(get_logger(), "CAN bus not initialized, ignore enable/disable command");
        return;
    }

    const int cmd = msg->data;
    if (cmd == 1)
    {
        // 逐个电机使能
        for (int i = 0; i < NumOfMotors; ++i)
        {
            canBus_->motorEnable(&motors_[i]);
        }
        RCLCPP_INFO(get_logger(), "All motors enabled via /motor/enable_cmd");
    }
    else if (cmd == 0)
    {
        // 逐个电机去使能
        for (int i = 0; i < NumOfMotors; ++i)
        {
            canBus_->motorDisable(&motors_[i]);
        }
        RCLCPP_INFO(get_logger(), "All motors disabled via /motor/enable_cmd");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Unknown enable_cmd value: %d (expect 0 or 1)", cmd);
    }
}

void MotorControlNode::setZeroCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!canBus_)
    {
        RCLCPP_WARN(get_logger(), "CAN bus not initialized, ignore set_zero command");
        return;
    }

    const int motor_id = msg->data;  // 1-6
    if (motor_id < 1 || motor_id > NumOfMotors)
    {
        RCLCPP_WARN(get_logger(), "Invalid motor id for set_zero: %d (expect 1-%d)",
                    motor_id, NumOfMotors);
        return;
    }

    const int idx = motor_id - 1;
    try
    {
        // 读取当前物理位置（弧度），作为新的逻辑零点
        const double current_pos = canBus_->GetPosition(&motors_[idx]);
        zero_offsets_[idx] = current_pos;
        RCLCPP_INFO(get_logger(), "Set motor %d current position %.3f rad as logical zero", motor_id, current_pos);
    }
    catch (...)
    {
        RCLCPP_WARN(get_logger(), "Failed to read motor %d position for set_zero", motor_id);
    }
}

void MotorControlNode::goHome()
{
    // 按要求禁用回零/回默认位姿
    RCLCPP_INFO(get_logger(), "goHome() skipped");
    return;

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

void MotorControlNode::publishJointStates()
{
    if (!canBus_)
        return;

    // 直接通过 SDO 方式依次读取每个电机的位置，避免依赖 TPDO 配置是否成功
    sensor_msgs::msg::JointState js;
    js.position.resize(NumOfMotors);

    for (int i = 0; i < NumOfMotors; ++i)
    {
        try
        {
            // GetPosition 返回的是物理弧度，减去零点偏移后作为“逻辑位置”发布
            js.position[i] = canBus_->GetPosition(&motors_[i]) - zero_offsets_[i];
        }
        catch (...)
        {
            // 若某个电机读取失败，则保持该关节为 0.0
            js.position[i] = 0.0;
        }
    }

    joint_state_pub_->publish(js);
}


} // namespace kvaser_motor_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvaser_motor_control::MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
