// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include "andino_base/kvaser.h"  // <- must expose KvaserForElmo/KvaserForGold/MOTOR and helpers
// 前向声明（来自 kvaser.h）
class KvaserForElmo;
class MOTOR;

namespace andino_base {

/// \brief 使用 Kvaser CAN 控制 Elmo 电机的 MotorDriver 封装。
/// 与原串口版本接口保持一致，以兼容上层程序。
class MotorDriver {
 public:
  /// @brief 编码器值类型（两个电机）。
  using Encoders = std::array<int, 2>;

  /// @brief 默认构造函数。
  MotorDriver();

  /// @brief 析构函数。
  ~MotorDriver();

  /// @brief 初始化 CAN 网络并配置电机。
  /// @param serial_device 保留参数（未使用，为兼容旧接口）
  /// @param baud_rate 保留参数（未使用，为兼容旧接口）
  /// @param timeout_ms 通信超时时间（毫秒）
  void Setup(const std::string& serial_device, int32_t baud_rate, int motor_id);

  /// @brief 发送空命令（保持接口一致，无操作）。
  void SendEmptyMsg();
  double get_position(int motor_id);   // ← 你要实现的
  double get_velocity(int motor_id);
  /// @brief 读取两个电机的编码器值。
  /// @return 左、右电机编码器值（整型）。
  Encoders ReadEncoderValues();

  /// @brief 设置两个电机的速度。
  /// @param val_1 左电机速度（单位：rad/s 或 counts/s）
  /// @param val_2 右电机速度（单位：rad/s 或 counts/s）
  void SetMotorValues(int val_1, int val_2);

  /// @brief 设置 PID 参数。
  /// @param k_p 比例增益
  /// @param k_d 微分增益
  /// @param k_i 积分增益
  /// @param k_o 偏置
  void SetPidValues(float k_p, float k_d, float k_i, float k_o);

  /// @brief 检查 CAN 网络是否已连接。
  bool is_connected() const;

  /// @brief 发送命令（内部实现基于 CAN 而非串口）。
  /// @param msg_to_send 命令字符串
  /// @return 响应字符串
  std::string SendMsg(const std::string& msg_to_send);

  /// @brief 设置 CAN 通道号，允许选择不同的 Kvaser 设备。
  /// @param can_channel Kvaser CAN 通道号（如 0、1 等）。
  void SetCanChannel(int can_channel);

 private:
  mutable std::mutex mutex_;  ///< 线程安全锁
  std::vector<MOTOR> motors_vec_;  // 存储电机对象，替代原来的局部数组
  MOTOR* motors_ptr_ = nullptr;    // 指向 motors_vec_ 的指针，供 kvaser_ 使用
  KvaserForGold* kvaser_ = nullptr; // CAN 总线对象

  // 选择使用的 Kvaser CAN 通道号（默认 0）
  int can_channel_{0};

  // 电机数量（通常为 2）
  int num_nodes_{6};

  // 节点 ID 列表（默认 [1,2]，可在 Setup 中修改）
  std::vector<int> node_ids_;
  std::vector<MOTOR> motors_;  // 用于存放 MOTOR 对象
  // 超时
  int32_t timeout_ms_{200};
};

}  // namespace andino_base
