
#include "andino_base/diffdrive_andino.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace andino_base {

hardware_interface::CallbackReturn DiffDriveAndino::on_init(const hardware_interface::HardwareInfo& info) 
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) 
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	RCLCPP_INFO(logger_, "On init...");

	config_.left_wheel_name = info_.hardware_parameters[kLeftWheelNameParam];
	RCLCPP_DEBUG(logger_, (kLeftWheelNameParam + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
	config_.right_wheel_name = info_.hardware_parameters[kRightWheelNameParam];
	RCLCPP_DEBUG(logger_, (kRightWheelNameParam + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());
	config_.serial_device = info_.hardware_parameters[kSerialDeviceParam];
	RCLCPP_DEBUG(logger_, (kSerialDeviceParam + static_cast<std::string>(": ") + config_.serial_device).c_str());
	config_.baud_rate = std::stoi(info_.hardware_parameters[kBaudRateParam]);
	RCLCPP_DEBUG(logger_,
				(kBaudRateParam + static_cast<std::string>(": ") + info_.hardware_parameters[kBaudRateParam]).c_str());
	config_.timeout = std::stoi(info_.hardware_parameters[kTimeoutParam]);
	RCLCPP_DEBUG(logger_,
				(kTimeoutParam + static_cast<std::string>(": ") + info_.hardware_parameters[kTimeoutParam]).c_str());
	config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters[kEncTicksPerRevParam]);
	RCLCPP_DEBUG(logger_,
				(kEncTicksPerRevParam + static_cast<std::string>(": ") + info_.hardware_parameters[kEncTicksPerRevParam])
					.c_str());

  // 可选的 CAN 通道号参数（默认为 0，兼容旧配置）
  auto it = info_.hardware_parameters.find(kCanChannelParam);
  if (it != info_.hardware_parameters.end()) {
    config_.can_channel = std::stoi(it->second);
    RCLCPP_DEBUG(
      logger_,
      (kCanChannelParam + static_cast<std::string>(": ") + it->second).c_str());
  } else {
    config_.can_channel = 0;
    RCLCPP_INFO(logger_, "Parameter '%s' not set, using default CAN channel %d", kCanChannelParam.c_str(), config_.can_channel);
  }

  // 左右电机 node ID（可选参数，默认 3/5，与现有实现一致）
  auto it_left_id = info_.hardware_parameters.find(kLeftMotorIdParam);
  if (it_left_id != info_.hardware_parameters.end())
  {
    config_.left_motor_id = std::stoi(it_left_id->second);
    RCLCPP_DEBUG(
      logger_,
      (kLeftMotorIdParam + static_cast<std::string>(": ") + it_left_id->second).c_str());
  }
  else
  {
    config_.left_motor_id = 7;
    RCLCPP_INFO(logger_, "Parameter '%s' not set, using default left motor id %d", kLeftMotorIdParam.c_str(), config_.left_motor_id);
  }

  auto it_right_id = info_.hardware_parameters.find(kRightMotorIdParam);
  if (it_right_id != info_.hardware_parameters.end())
  {
    config_.right_motor_id = std::stoi(it_right_id->second);
    RCLCPP_DEBUG(
      logger_,
      (kRightMotorIdParam + static_cast<std::string>(": ") + it_right_id->second).c_str());
  }
  else
  {
    config_.right_motor_id = 8;
    RCLCPP_INFO(logger_, "Parameter '%s' not set, using default right motor id %d", kRightMotorIdParam.c_str(), config_.right_motor_id);
  }

	for (const hardware_interface::ComponentInfo& joint : info.joints) {
		// DiffDriveAndino has exactly two states and one command interface on each joint
		if (joint.command_interfaces.size() != 1) {
		RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
					joint.command_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces.size() != 2) {
		RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
					joint.state_interfaces.size());
		return hardware_interface::CallbackReturn::ERROR;
		}
	}

	// Set up the wheels
	left_wheel_.Setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
	right_wheel_.Setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

	RCLCPP_INFO(logger_, "Finished On init.");

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "On configure...");

  // Set up communication with motor driver controller.
  // 阶段 1：设置通道，注册电机（此时不创建 Kvaser，不操作 CAN）
  motor_driver_.SetCanChannel(config_.can_channel);
  motor_driver_.Setup(config_.serial_device, config_.baud_rate, config_.left_motor_id);
  motor_driver_.Setup(config_.serial_device, config_.baud_rate, config_.right_motor_id);

  // 阶段 2：设置左右电机 ID 和方向（此时 vector 不再增长，指针稳定）
  motor_driver_.SetMotorIds(config_.left_motor_id, config_.right_motor_id);

  // 阶段 3：创建 KvaserForGold 并完成所有 CAN 配置
  motor_driver_.Connect();
  RCLCPP_INFO(logger_, "Finished Configuration");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveAndino::export_state_interfaces() {
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // TODO(francocipollone): Probably we could use the information obtained via info_ variable about the joint name
  // directly.
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveAndino::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On activate...");
  RCLCPP_INFO(logger_, "Finished Activation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On deactivate...");
  RCLCPP_INFO(logger_, "Finished Deactivation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveAndino::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // 一次性通过 TPDO 刷新所有电机的位置/速度缓存，确保左右轮数据同步
  motor_driver_.UpdateSensorData();

  left_wheel_.pos_= motor_driver_.get_position(config_.left_motor_id);
  left_wheel_.vel_ = motor_driver_.get_velocity(config_.left_motor_id);

  right_wheel_.pos_ = motor_driver_.get_position(config_.right_motor_id);
  right_wheel_.vel_ = motor_driver_.get_velocity(config_.right_motor_id);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveAndino::write(const rclcpp::Time& /* time */,
                                                       const rclcpp::Duration& /* period */) {
  if (!motor_driver_.is_connected()) {
    RCLCPP_ERROR(logger_, "Motor driver is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  // The command from the controller is in rad/sec. We forward it as-is
  // to the motor driver so that small angular velocities are preserved.

  const double left_value_target = left_wheel_.cmd_;
  const double right_value_target = right_wheel_.cmd_;
  RCLCPP_DEBUG(logger_, "Target speed - Left: %.3f, Right: %.3f", left_value_target, right_value_target);
  motor_driver_.SetMotorValues(left_value_target, right_value_target);

  return hardware_interface::return_type::OK;
}

}  // namespace andino_base

PLUGINLIB_EXPORT_CLASS(andino_base::DiffDriveAndino, hardware_interface::SystemInterface)
