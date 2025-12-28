#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "lifter_modbus/lifter.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto logger = rclcpp::get_logger("lifter_test");

	lifter_modbus::Lifter lifter;

	if (!lifter.initModbus())
	{
		RCLCPP_ERROR(logger, "Failed to initialize Modbus from YAML config");
		rclcpp::shutdown();
		return 1;
	}

	RCLCPP_INFO(logger, "Modbus initialized, enabling lifter...");
	if (!lifter.enable())
	{
		RCLCPP_ERROR(logger, "Failed to enable lifter");
		rclcpp::shutdown();
		return 1;
	}

	// 测试流程（单位：mm）：
	// 1) 位置模式移动到 80 mm 处
	// 2) 间隔 2s
	// 3) 移动到 40 mm 处
	// 4) 再重复一次 80 mm -> 40 mm
	// 5) 最后回到 0 mm 处停止，然后失能

	auto move_and_wait = [&](std::int32_t target_mm, std::chrono::milliseconds wait_ms) {
		RCLCPP_INFO(logger, "Move to %d mm", target_mm);
		if (!lifter.moveToPosition(static_cast<double>(target_mm)))
		{
			RCLCPP_ERROR(logger, "moveToPosition(%d mm) failed", target_mm);
			return false;
		}
		std::this_thread::sleep_for(wait_ms);
		std::int32_t pos_mm = lifter.getCurrentPosition();
		RCLCPP_INFO(logger, "Current position after move: %d mm", pos_mm);
		return true;
	};

	// 在 80 mm 与 40 mm 之间往复运行，直到被 Ctrl+C 中断
	while (rclcpp::ok())
	{
		if (!move_and_wait(80, 2000ms))
		{
			goto shutdown;
		}
		if (!move_and_wait(40, 2000ms))
		{
			goto shutdown;
		}
	}

	// 退出前回到 0 mm
	if (!move_and_wait(0, 2000ms))
	{
		goto shutdown;
	}

	RCLCPP_INFO(logger, "Disabling lifter...");
	if (!lifter.disable())
	{
		RCLCPP_WARN(logger, "Failed to disable lifter");
	}
	else
	{
		RCLCPP_INFO(logger, "Lifter disabled");
	}

shutdown:
	rclcpp::shutdown();
	return 0;
}
