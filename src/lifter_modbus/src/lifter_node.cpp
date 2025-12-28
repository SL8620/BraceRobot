#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "lifter_modbus/lifter.hpp"

namespace lifter_modbus
{

class LifterNode : public rclcpp::Node
{
public:
	LifterNode()
	    : rclcpp::Node("lifter_node")
	{
		pos_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lifter_pos", 10);
		status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lifter_status", 10);

		cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
		    "/lifter_cmd", 10,
		    std::bind(&LifterNode::cmdCallback, this, std::placeholders::_1));

		// 初始化 Modbus，并尝试使能电机
		if (!lifter_.initModbus())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize Modbus from YAML config");
			initialized_ = false;
		}
		else
		{
			initialized_ = true;
			if (lifter_.enable())
			{
				enabled_ = true;
				RCLCPP_INFO(this->get_logger(), "Lifter enabled successfully");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Lifter enable command failed");
			}
		}

		// 50Hz 定时器，发布当前位置和状态
		using namespace std::chrono_literals;
		timer_ = this->create_wall_timer(
		    20ms, std::bind(&LifterNode::timerCallback, this));
	}

	~LifterNode() override
	{
		if (initialized_ && enabled_)
		{
			RCLCPP_INFO(this->get_logger(), "Disabling lifter on node shutdown...");
			if (!lifter_.disable())
			{
				RCLCPP_WARN(this->get_logger(), "Failed to disable lifter during shutdown");
			}
		}
	}

private:
	void cmdCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
	{
		if (!initialized_)
		{
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			                    "Lifter not initialized, ignore /lifter_cmd");
			return;
		}

		if (msg->data.empty())
		{
			RCLCPP_WARN(this->get_logger(), "/lifter_cmd message requires at least 1 int: command type");
			return;
		}

		int cmd_type = msg->data[0];
		bool ok = false;

		switch (cmd_type)
		{
		case 0: // 位置模式，data[1] = 目标高度(mm)
			if (msg->data.size() < 2)
			{
				RCLCPP_WARN(this->get_logger(), "/lifter_cmd type 0 requires data[1]=target position (mm)");
				return;
			}
			else
			{
				int32_t target_mm = msg->data[1];
				ok = lifter_.moveToPosition(static_cast<double>(target_mm));
				if (!ok)
				{
					RCLCPP_WARN(this->get_logger(), "Lifter position command failed: target=%d mm", target_mm);
				}
			}
			break;
		case 1: // 使能
			ok = lifter_.enable();
			if (ok)
			{
				enabled_ = true;
				RCLCPP_INFO(this->get_logger(), "Lifter enabled by command");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Lifter enable command failed");
			}
			break;
		case 2: // 失能
			ok = lifter_.disable();
			if (ok)
			{
				enabled_ = false;
				RCLCPP_INFO(this->get_logger(), "Lifter disabled by command");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Lifter disable command failed");
			}
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "Unknown /lifter_cmd type: %d (expected 0=pos,1=enable,2=disable)", cmd_type);
			return;
		}
	}

	void timerCallback()
	{
		// 发布当前位置
		std_msgs::msg::Float32 pos_msg;
		if (initialized_)
		{
			std::int32_t pos_mm = lifter_.getCurrentPosition();
			pos_msg.data = static_cast<float>(pos_mm);
		}
		else
		{
			pos_msg.data = 0.0f;
		}
		pos_pub_->publish(pos_msg);

		// 发布使能状态（0/1）
		std_msgs::msg::Int32 status_msg;
		status_msg.data = enabled_ ? 1 : 0;
		status_pub_->publish(status_msg);
	}

	Lifter lifter_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool initialized_ = false;
	bool enabled_ = false;
};

} // namespace lifter_modbus

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<lifter_modbus::LifterNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

