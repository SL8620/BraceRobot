// motor_driver_kvaser.cpp
// Based on and replacing the serial-based implementation with Kvaser CAN-based control.
// Assumptions:
//  - Two motors, node IDs 1 and 2 (adjust node_ids_ if different).
//  - SetMotorValues(val1,val2) expects velocities in rad/s (same external API as before).
//  - ReadEncoderValues() returns integer positions in milliradians (pos_rad * 1000 -> int).
//  - Requires your kvaser.h/cpp compiled and available in include/linker paths.
//  - Uses KvaserForElmo (or KvaserForGold) interfaces present in your provided file.
//  - If you want counts instead of milliradians, we can adapt using rad2cnt() and the MOTOR.encoder fields.

#include "andino_base/motor_driver.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <sstream>
#include <algorithm>


namespace andino_base {

MotorDriver::MotorDriver() : kvaser_(nullptr), can_channel_(0), num_nodes_(0), timeout_ms_(200)
{
}

MotorDriver::~MotorDriver() 
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (kvaser_) 
	{
		// attempt to gracefully disable motors
		try 
		{
			kvaser_->DisableMotors();
		} 
		catch (...) 
		{
			// ignore
		}
		delete kvaser_;
		kvaser_ = nullptr;
	}
}

void MotorDriver::SetCanChannel(int can_channel)
{
	std::lock_guard<std::mutex> lock(mutex_);
	can_channel_ = can_channel;
}

void MotorDriver::SetMotorIds(int left_motor_id, int right_motor_id)
{
	std::lock_guard<std::mutex> lock(mutex_);
	left_motor_id_ = left_motor_id;
	right_motor_id_ = right_motor_id;

	// 左轮电机镜像安装，需要反向。通过 direction 字段统一处理，
	// 使 rad2cnt / cnt2rad 自动完成符号翻转，read/write 方向对称。
	for (int i = 0; i < num_nodes_; ++i) {
		if (motors_ptr_ && motors_ptr_[i].id == left_motor_id) {
			motors_ptr_[i].direction = -1;
		}
	}
}

void MotorDriver::Setup(const std::string& , int32_t /*baud_rate*/,int motor_id) 
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查是否已存在
    for (auto& m : motors_vec_) 
	{
        if (m.id == motor_id) return; // 已经初始化过了
    }

    motors_vec_.push_back(MOTOR{});
    MOTOR& m = motors_vec_.back();
    m.id = motor_id;
    m.connect = true;
    m.Kt_inv = 0.1;
    m.In = 10.0;
    m.Wn = 50.0;
    m.direction = 1;
    m.encoder.count = 131072;
    m.encoder.AbsZeroPos = 0;
    m.InitPos = 0.0;

	motors_ptr_ = motors_vec_.data();
	num_nodes_ = motors_vec_.size();

    std::cout << "电机 " << m.id << " 已注册 (共 " << num_nodes_ << " 台)" << std::endl;
}

void MotorDriver::Connect()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!motors_ptr_ || num_nodes_ == 0) 
	{
        std::cerr << "No motors registered, call Setup() first." << std::endl;
        return;
    }

    // 确保 motors_ptr_ 指向最新的 vector 数据（之后不再 push_back）
    motors_ptr_ = motors_vec_.data();

    if (!kvaser_) 
	{
        kvaser_ = new KvaserForGold(can_channel_, num_nodes_, motors_ptr_, "ChassisMotor");
    } 
	else 
	{
        // 同步指针和数量，防止悬空
        kvaser_->pNode = motors_ptr_;
        kvaser_->NumOfNodes = num_nodes_;
    }

    for (int i = 0; i < num_nodes_; ++i) 
	{
        MOTOR* m = &motors_ptr_[i];
        if (!m->connect) continue;
        kvaser_->connectMotor(m);
        kvaser_->RPDOconfig(m, KvaserForGold::SPEED_MODE);
        kvaser_->TPDOconfigPXVX(m, 2);
        kvaser_->modeChoose(m, KvaserForGold::SPEED_MODE);
        std::cout << "电机 " << m->id << " 已进入速度模式。" << std::endl;
    }
}

bool MotorDriver::is_connected() const 
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!kvaser_) return false;
	// Kvaser class doesn't expose IsOpen; we assume initialization success implies connection.
	return true;
}

void MotorDriver::SendEmptyMsg() 
{
	SendMsg("");
}

void MotorDriver::SetMotorValues(double val_left, double val_right) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!kvaser_ || !motors_ptr_) 
	{
        std::cerr << "Kvaser not initialized; cannot set motor values." << std::endl;
        return;
    }

    // 先将目标速度写到每个电机的 jv 字段
    for (int i = 0; i < num_nodes_; ++i) 
	{
		MOTOR* m = &motors_ptr_[i];
        if (!m->connect) continue;
		if (m->id == left_motor_id_) 
		{
			m->jv = val_left;
		} 
		else if (m->id == right_motor_id_) 
		{
			m->jv = val_right;
		}
    }

    // 一次性同步发出所有电机速度命令（canWrite + canWriteSync）
    kvaser_->SendSpeedCommandForAll();
}


void MotorDriver::SetPidValues(float k_p, float k_d, float k_i, float k_o) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!kvaser_ || !motors_ptr_) {
    std::cerr << "Kvaser not initialized; cannot set PID values." << std::endl;
    return;
  }


  for (int i = 0; i < num_nodes_; ++i) 
  {
    MOTOR* m = &motors_ptr_[i];
    if (!m->connect) continue;
    std::cout << "SetPidValues: motor " << m->id << " Kp=" << k_p << " Kd=" << k_d  << " Ki=" << k_i << " Ko=" << k_o << std::endl;
  }
}

std::string MotorDriver::SendMsg(const std::string& msg) 
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!kvaser_) 
	{
		return std::string("Kvaser_not_initialized");
	}

	if (msg.empty()) 
	{
		return std::string("OK");
	}

	std::istringstream iss(msg);
	std::string cmd;
	iss >> cmd;
	if (cmd == "e") 
	{
		Encoders e = ReadEncoderValues();
		std::ostringstream oss;
		oss << e[0] << " " << e[1];
		return oss.str();
	} 
	else if (cmd == "m") 
	{
		double v1 = 0.0, v2 = 0.0;
		iss >> v1 >> v2;
		SetMotorValues(v1, v2);
		return std::string("OK");
	} 
	else if (cmd == "u") 
	{
		std::string rest;
		if (std::getline(iss, rest)) 
		{
			// rest like " 1.0:0.1:0.01:0.0"
			float kp=0,kd=0,ki=0,ko=0;
			std::replace(rest.begin(), rest.end(), ':', ' ');
			std::istringstream s2(rest);
			s2 >> kp >> kd >> ki >> ko;
			SetPidValues(kp,kd,ki,ko);
			return std::string("OK");
		}
		return std::string("ERR");
	}

	return std::string("UNSUPPORTED_CMD");
}

MotorDriver::Encoders MotorDriver::ReadEncoderValues() 
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!kvaser_ || !motors_ptr_) 
	{
		return {0, 0};
	}

	int out1 = 0, out2 = 0;
	for (int i = 0; i < num_nodes_; ++i) 
	{
		MOTOR* m = &motors_ptr_[i];
		if (!m->connect) continue;

		double pos_rad = kvaser_->GetPosition(m);
		if (std::isnan(pos_rad)) 
		{
			// fallback: try GetVelocity or zero
			pos_rad = 0.0;
		}
		int milli_rad = static_cast<int>(pos_rad * 1000.0); // milliradians -> int
		if (i == 0) out1 = milli_rad;
		else out2 = milli_rad;
	}

	return {out1, out2};
}

void MotorDriver::UpdateSensorData()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!kvaser_ || !motors_ptr_)
        return;

    // 通过 TPDO 一次性读取所有电机的位置和速度到 motor->px / motor->vx
    kvaser_->GetPositionAndVelocity();
}

double MotorDriver::get_position(int motor_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!kvaser_ || !motors_ptr_) 
	{
        std::cerr << "Kvaser not initialized!" << std::endl;
        return 0.0;
    }

    // 直接返回 TPDO 缓存的位置值（由 UpdateSensorData 统一刷新）
    for (int i = 0; i < num_nodes_; ++i) 
	{
        MOTOR* m = &motors_ptr_[i];
        if (!m->connect) continue;
        if (m->id == motor_id) 
		{
            return m->px;
        }
    }
    std::cerr << "Motor ID " << motor_id << " not found!" << std::endl;
    return 0.0;
}

double MotorDriver::get_velocity(int motor_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!kvaser_ || !motors_ptr_) 
	{
        std::cerr << "Kvaser not initialized!" << std::endl;
        return 0.0;
    }

    // 直接返回 TPDO 缓存的速度值（由 UpdateSensorData 统一刷新）
    for (int i = 0; i < num_nodes_; ++i) 
	{
        MOTOR* m = &motors_ptr_[i];
        if (!m->connect) continue;
        if (m->id == motor_id) 
		{
            return m->vx;
        }
    }

    std::cerr << "Motor ID " << motor_id << " not found!" << std::endl;
    return 0.0;
}

}  // namespace andino_base
