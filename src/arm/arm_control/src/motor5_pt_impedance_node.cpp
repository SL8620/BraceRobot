#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "kvaser.h"

using namespace std::chrono_literals;

namespace kvaser_motor_control
{

class Motor5PtImpedanceNode final : public rclcpp::Node
{
public:
    Motor5PtImpedanceNode()
    : Node("motor5_pt_impedance_node")
    {
        can_channel_ = this->declare_parameter<int>("can_channel", 0);
        motor_id_ = this->declare_parameter<int>("motor_id", 5);
        control_period_ms_ = this->declare_parameter<double>("control_period_ms", 20.0);

        kp_ = this->declare_parameter<double>("kp", 0.0);
        kd_ = this->declare_parameter<double>("kd", 0.0);

        hold_current_on_start_ = this->declare_parameter<bool>("hold_current_on_start", true);
        max_torque_ = this->declare_parameter<double>("max_torque", 0.0); // 0 = no clamp

        enabled_ = this->declare_parameter<bool>("enabled", true);

        // Desired setpoints (rad, rad/s)
        qd_ = this->declare_parameter<double>("qd", 0.0);
        qd_dot_ = this->declare_parameter<double>("qd_dot", 0.0);

        RCLCPP_INFO(get_logger(), "Motor5 PT Impedance node starting (CAN=%d, motor_id=%d)", can_channel_, motor_id_);

        initMotorStruct();
        initCanAndMotor();

        // If requested, capture current position as initial target to avoid a step.
        if (hold_current_on_start_)
        {
            qd_ = can_bus_->GetPosition(&motors_[1]);
            qd_dot_ = 0.0;
            RCLCPP_INFO(get_logger(), "hold_current_on_start=true -> qd set to current position: %.6f rad", qd_);
        }

        qd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor5/impedance_position_cmd", 10,
            std::bind(&Motor5PtImpedanceNode::qdCallback, this, std::placeholders::_1));

        qd_dot_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor5/impedance_velocity_cmd", 10,
            std::bind(&Motor5PtImpedanceNode::qdDotCallback, this, std::placeholders::_1));

        // Per-motor enable/disable: 1=enable, 0=disable
        enable_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/motor5/enable_cmd", 10,
            std::bind(&Motor5PtImpedanceNode::enableCallback, this, std::placeholders::_1));

        // State publisher: [p, v, t, pd, vd, kp, kd, enabled]
        state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor5/pt_impedance_state", 10);

        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Motor5PtImpedanceNode::onSetParameters, this, std::placeholders::_1));

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::milli>(control_period_ms_));
        timer_ = this->create_wall_timer(period, std::bind(&Motor5PtImpedanceNode::controlLoop, this));
    }

    ~Motor5PtImpedanceNode() override
    {
        try
        {
            if (can_bus_)
            {
                can_bus_->motorDisable(&motors_[1]);
                can_bus_->canRelease();
            }
        }
        catch (...) {}
    }

private:
    void initMotorStruct()
    {
        // IMPORTANT: KvaserForElmo/KvaserForGold internally uses pNode[1..NumOfNodes].
        // So we keep index 0 as a dummy and store the motor at index 1.
        motors_.fill(MOTOR{});

        motors_[1].id = motor_id_;
        motors_[1].connect = true;
        motors_[1].Kt_inv = 0.1;
        motors_[1].In = 10.0;
        motors_[1].Wn = 50.0;
        motors_[1].direction = 1;
        motors_[1].encoder.count = 131072;
        motors_[1].encoder.AbsZeroPos = 0;
        motors_[1].InitPos = 0.0;
    }

    void initCanAndMotor()
    {
        can_bus_ = std::make_shared<KvaserForGold>(can_channel_, 1, motors_.data(), "Motor5PtImpedance");

        // Connect + configure only this motor.
        can_bus_->connectMotor(&motors_[1]);
        can_bus_->RPDOconfig(&motors_[1], KvaserForGold::TORQUE_MODE);
        can_bus_->modeChoose(&motors_[1], KvaserForGold::TORQUE_MODE);
        can_bus_->TPDOconfigPXVX(&motors_[1], 2);

        RCLCPP_INFO(get_logger(), "Motor %d initialized in TORQUE (PT) mode", motor_id_);
    }

    void qdCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        qd_ = msg->data;
    }

    void qdDotCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        qd_dot_ = msg->data;
    }

    void enableCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!can_bus_)
            return;

        const int cmd = msg->data;
        if (cmd == 1)
        {
            can_bus_->motorEnable(&motors_[1]);
            enabled_ = true;
            RCLCPP_INFO(get_logger(), "Motor %d enabled via /motor5/enable_cmd", motor_id_);
        }
        else if (cmd == 0)
        {
            // Send zero torque once before disabling for safety.
            can_bus_->SendTorqueCommand(&motors_[1], 0.0);
            can_bus_->motorDisable(&motors_[1]);
            enabled_ = false;
            RCLCPP_INFO(get_logger(), "Motor %d disabled via /motor5/enable_cmd", motor_id_);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Invalid /motor5/enable_cmd=%d (use 1 enable, 0 disable)", cmd);
        }
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "kp")
                kp_ = p.as_double();
            else if (p.get_name() == "kd")
                kd_ = p.as_double();
            else if (p.get_name() == "qd")
                qd_ = p.as_double();
            else if (p.get_name() == "qd_dot")
                qd_dot_ = p.as_double();
            else if (p.get_name() == "max_torque")
                max_torque_ = p.as_double();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void controlLoop()
    {
        if (!can_bus_)
            return;

        if (!enabled_)
            return;

        const double q = can_bus_->GetPosition(&motors_[1]);
        const double qdot = can_bus_->GetVelocity(&motors_[1]);

        double tau = kp_ * (qd_ - q) + kd_ * (qd_dot_ - qdot);

        if (max_torque_ > 0.0)
        {
            if (tau > max_torque_) tau = max_torque_;
            if (tau < -max_torque_) tau = -max_torque_;
        }

        can_bus_->SendTorqueCommand(&motors_[1], tau);

        if (state_pub_)
        {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.resize(8);
            msg.data[0] = q;
            msg.data[1] = qdot;
            msg.data[2] = tau;
            msg.data[3] = qd_;
            msg.data[4] = qd_dot_;
            msg.data[5] = kp_;
            msg.data[6] = kd_;
            msg.data[7] = enabled_ ? 1.0 : 0.0;
            state_pub_->publish(msg);
        }

        // Throttle status logs
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "q=%.4f rad qd=%.4f rad | qdot=%.4f rad/s | kp=%.2f kd=%.2f | tau=%.4f Nm",
            q, qd_, qdot, kp_, kd_, tau);
    }

private:
    int can_channel_{0};
    int motor_id_{5};
    double control_period_ms_{20.0};

    double kp_{0.0};
    double kd_{0.0};
    double qd_{0.0};
    double qd_dot_{0.0};
    bool hold_current_on_start_{true};
    double max_torque_{0.0};

    bool enabled_{true};

    std::array<MOTOR, 2> motors_{};
    std::shared_ptr<KvaserForGold> can_bus_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr qd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr qd_dot_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enable_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

} // namespace kvaser_motor_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvaser_motor_control::Motor5PtImpedanceNode>());
    rclcpp::shutdown();
    return 0;
}
