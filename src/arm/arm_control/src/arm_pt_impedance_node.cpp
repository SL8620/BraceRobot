#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>

#include "kvaser.h"

using namespace std::chrono_literals;

namespace kvaser_motor_control
{

class ArmPtImpedanceNode final : public rclcpp::Node
{
public:
    static constexpr int NumMotors = 6;

    ArmPtImpedanceNode()
    : Node("arm_pt_impedance_node")
    {
        can_channel_ = this->declare_parameter<int>("can_channel", 0);
        control_period_ms_ = this->declare_parameter<double>("control_period_ms", 20.0);
        hold_current_on_start_ = this->declare_parameter<bool>("hold_current_on_start", true);

        kp_ = this->declare_parameter<std::vector<double>>("kp", std::vector<double>(NumMotors, 0.0));
        kd_ = this->declare_parameter<std::vector<double>>("kd", std::vector<double>(NumMotors, 0.0));
        max_torque_ = this->declare_parameter<std::vector<double>>("max_torque", std::vector<double>(NumMotors, 0.0));
        enabled_ = this->declare_parameter<std::vector<bool>>("enabled", std::vector<bool>(NumMotors, true));

        qd_.assign(NumMotors, 0.0);
        qd_dot_.assign(NumMotors, 0.0);

        RCLCPP_INFO(get_logger(), "Arm PT Impedance starting (CAN=%d)", can_channel_);

        initMotorStructs();
        initCanAndMotors();

        if (hold_current_on_start_)
        {
            for (int i = 0; i < NumMotors; ++i)
            {
                try
                {
                    qd_[i] = can_bus_->GetPosition(&motors_[i]);
                    qd_dot_[i] = 0.0;
                }
                catch (...) {}
            }
            RCLCPP_INFO(get_logger(), "hold_current_on_start=true -> qd initialized to current positions");
        }

        impedance_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor/impedance_position_cmd", 10,
            std::bind(&ArmPtImpedanceNode::impedanceCmdCallback, this, std::placeholders::_1));

        enable_all_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor/enable_cmd", 10,
            std::bind(&ArmPtImpedanceNode::enableAllCallback, this, std::placeholders::_1));

        enable_one_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor/enable_one_cmd", 10,
            std::bind(&ArmPtImpedanceNode::enableOneCallback, this, std::placeholders::_1));

        motor6_hw_zero_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor6/set_zero_hw", 10,
            std::bind(&ArmPtImpedanceNode::motor6SetZeroHwCallback, this, std::placeholders::_1));

        state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor/pt_impedance_state", 10);

        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArmPtImpedanceNode::onSetParameters, this, std::placeholders::_1));

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::milli>(control_period_ms_));
        timer_ = this->create_wall_timer(period, std::bind(&ArmPtImpedanceNode::controlLoop, this));
    }

    ~ArmPtImpedanceNode() override
    {
        try
        {
            if (can_bus_)
            {
                for (int i = 0; i < NumMotors; ++i)
                    can_bus_->SendTorqueCommand(&motors_[i], 0.0);
                for (int i = 0; i < NumMotors; ++i)
                    can_bus_->motorDisable(&motors_[i]);
                can_bus_->canRelease();
            }
        }
        catch (...) {}
    }

private:
    void initMotorStructs()
    {
        for (int i = 0; i < NumMotors; i++)
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

    void initCanAndMotors()
    {
        can_bus_ = std::make_shared<KvaserForGold>(can_channel_, NumMotors, motors_, "ArmPtImpedance");

        for (int i = 0; i < NumMotors; i++)
        {
            can_bus_->connectMotor(&motors_[i]);
            can_bus_->RPDOconfig(&motors_[i], KvaserForGold::TORQUE_MODE);
            can_bus_->modeChoose(&motors_[i], KvaserForGold::TORQUE_MODE);
            can_bus_->TPDOconfigPXVX(&motors_[i], 2);
        }

        RCLCPP_INFO(get_logger(), "Motors initialized in TORQUE (PT) mode");
    }

    void impedanceCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() >= NumMotors)
        {
            for (int i = 0; i < NumMotors; i++)
                qd_[i] = msg->position[i];
        }

        if (msg->velocity.size() >= NumMotors)
        {
            for (int i = 0; i < NumMotors; i++)
                qd_dot_[i] = msg->velocity[i];
        }
    }

    void enableAllCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!can_bus_)
            return;

        const int cmd = msg->data;
        if (cmd == 1)
        {
            for (int i = 0; i < NumMotors; ++i)
            {
                can_bus_->motorEnable(&motors_[i]);
                if (i < (int)enabled_.size())
                    enabled_[i] = true;
            }
            RCLCPP_INFO(get_logger(), "All motors enabled via /motor/enable_cmd");
        }
        else if (cmd == 0)
        {
            for (int i = 0; i < NumMotors; ++i)
            {
                can_bus_->SendTorqueCommand(&motors_[i], 0.0);
                can_bus_->motorDisable(&motors_[i]);
                if (i < (int)enabled_.size())
                    enabled_[i] = false;
            }
            RCLCPP_INFO(get_logger(), "All motors disabled via /motor/enable_cmd");
        }
    }

    void enableOneCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!can_bus_)
            return;

        const int cmd = msg->data;
        if (cmd == 0)
            return;

        const int motor_id = std::abs(cmd);
        if (motor_id < 1 || motor_id > NumMotors)
            return;

        const int idx = motor_id - 1;
        if (cmd > 0)
        {
            can_bus_->motorEnable(&motors_[idx]);
            if (idx < (int)enabled_.size())
                enabled_[idx] = true;
            RCLCPP_INFO(get_logger(), "Motor %d enabled via /motor/enable_one_cmd", motor_id);
        }
        else
        {
            can_bus_->SendTorqueCommand(&motors_[idx], 0.0);
            can_bus_->motorDisable(&motors_[idx]);
            if (idx < (int)enabled_.size())
                enabled_[idx] = false;
            RCLCPP_INFO(get_logger(), "Motor %d disabled via /motor/enable_one_cmd", motor_id);
        }
    }

    void motor6SetZeroHwCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!can_bus_)
            return;

        if (msg->data != 1)
            return;

        // Motor 6 index = 5
        MOTOR* m6 = &motors_[5];

        // DS402: 0x6064 (Position actual value), 0x607C (Home offset, i32)
        // Manual definition:
        //   HomeOffset = ZeroPosition - HomeAttainedPosition
        // If we want ZeroPosition=0 and we treat the current position as the reference (home attained),
        // then we should write HomeOffset = -PositionActual.
        int32_t pos_cnt = 0;
        if (!can_bus_->SDOReadI32(m6, 0x6064, 0x00, pos_cnt, 200))
        {
            RCLCPP_WARN(get_logger(), "Motor6 hw zero: SDOReadI32(0x6064) failed");
            return;
        }

        int32_t old_home_off = 0;
        (void)can_bus_->SDOReadI32(m6, 0x607C, 0x00, old_home_off, 200);

        const int32_t new_home_off = -pos_cnt;

        // Safety: zero torque and disable before writing
        can_bus_->SendTorqueCommand(m6, 0.0);
        can_bus_->motorDisable(m6);

        const bool ok = can_bus_->SDOWriteI32(m6, 0x607C, 0x00, new_home_off, 200);
        if (!ok)
        {
            RCLCPP_WARN(get_logger(), "Motor6 hw zero: SDOWriteI32(0x607C) failed");
            can_bus_->motorEnable(m6);
            return;
        }

        can_bus_->motorEnable(m6);

        RCLCPP_INFO(
            get_logger(),
            "Motor6 hw zero OK: wrote 0x607C:00 home_offset %d -> %d using pos_actual(0x6064)=%d (effect depends on drive/homing)",
            old_home_off, new_home_off, pos_cnt);
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "kp")
                kp_ = p.as_double_array();
            else if (p.get_name() == "kd")
                kd_ = p.as_double_array();
            else if (p.get_name() == "max_torque")
                max_torque_ = p.as_double_array();
            else if (p.get_name() == "enabled")
                enabled_ = p.as_bool_array();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    double getVec(const std::vector<double>& v, int idx) const
    {
        if (idx < 0 || idx >= (int)v.size())
            return 0.0;
        return v[idx];
    }

    bool getEnabled(int idx) const
    {
        if (idx < 0 || idx >= (int)enabled_.size())
            return true;
        return enabled_[idx];
    }

    void controlLoop()
    {
        if (!can_bus_)
            return;

        sensor_msgs::msg::JointState st;
        st.position.resize(NumMotors);
        st.velocity.resize(NumMotors);
        st.effort.resize(NumMotors);

        for (int i = 0; i < NumMotors; i++)
        {
            if (!getEnabled(i))
            {
                can_bus_->SendTorqueCommand(&motors_[i], 0.0);
                st.position[i] = 0.0;
                st.velocity[i] = 0.0;
                st.effort[i] = 0.0;
                continue;
            }

            double q = 0.0;
            double qdot = 0.0;
            try
            {
                q = can_bus_->GetPosition(&motors_[i]);
                qdot = can_bus_->GetVelocity(&motors_[i]);
            }
            catch (...)
            {
                can_bus_->SendTorqueCommand(&motors_[i], 0.0);
                st.position[i] = 0.0;
                st.velocity[i] = 0.0;
                st.effort[i] = 0.0;
                continue;
            }

            const double tau = getVec(kp_, i) * (qd_[i] - q) + getVec(kd_, i) * (qd_dot_[i] - qdot);
            double tau_cmd = tau;
            const double clamp = getVec(max_torque_, i);
            if (clamp > 0.0)
            {
                if (tau_cmd > clamp) tau_cmd = clamp;
                if (tau_cmd < -clamp) tau_cmd = -clamp;
            }

            can_bus_->SendTorqueCommand(&motors_[i], tau_cmd);

            st.position[i] = q;
            st.velocity[i] = qdot;
            st.effort[i] = tau_cmd;
        }

        state_pub_->publish(st);
    }

private:
    int can_channel_{0};
    double control_period_ms_{20.0};
    bool hold_current_on_start_{true};

    MOTOR motors_[NumMotors]{};
    std::shared_ptr<KvaserForGold> can_bus_;

    std::vector<double> kp_;
    std::vector<double> kd_;
    std::vector<double> max_torque_;
    std::vector<bool> enabled_;

    std::vector<double> qd_;
    std::vector<double> qd_dot_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr impedance_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enable_all_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enable_one_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor6_hw_zero_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

} // namespace kvaser_motor_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvaser_motor_control::ArmPtImpedanceNode>());
    rclcpp::shutdown();
    return 0;
}
