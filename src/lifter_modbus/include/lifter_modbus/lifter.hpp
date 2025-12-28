#pragma once

#include <cstdint>
#include <string>

namespace lifter_modbus
{

class Lifter
{
public:
	Lifter();
	~Lifter();

	// 初始化 Modbus 设备：使用在构造函数中从 YAML 读取的配置
	// 包括 device、baudrate、parity、stopbits、slave_id
	bool initModbus(); 

	// 使能驱动器
	bool enable();

	// 失能驱动器
	bool disable();

	// 设置当前机械位置为零点
	bool setZero();

	// 位置模式运动到目标位置
	// target_position: 目标电缸位置，单位 mm，要求 min_height_ <= target_position <= max_height_
	// 内部按线性关系换算为电机位置（cnt）：
	//   0 mm -> 0 cnt, 95 mm -> 10,000,000 cnt
	bool moveToPosition(double target_position);

	// 读取当前位置（32 位有符号整型，单位：mm，按线性关系由电机 cnt 换算并四舍五入）
	std::int32_t getCurrentPosition() const;

	// 读取驱动器状态（可返回原始状态字/状态码）
	std::uint16_t getDriveStatus() const;

	// 打断正在执行的位置指令（写入执行寄存器 0x0919 = 0x0000）
	bool abortPositionCommand();

	// 发送原始 Modbus RTU 请求帧（frame 不含 CRC，函数内部自动附加 CRC 并通过串口发送）
	// frame: 请求数据（地址 + 功能码 + 数据），length: 字节数，返回 true 表示发送成功
	bool sendRawRequest(const std::uint8_t *frame, int length) const;

	// 接收 Modbus RTU 响应帧（从串口读取完整一帧，包括 CRC）
	// 返回接收到的字节数，失败返回 -1
	int receiveResponse(std::uint8_t *buffer, int max_length) const;

	// 生成 Modbus 报文：输入 6 字节数据帧，输出附带 CRC16 校验的 8 字节帧
	// in_frame[0..5] -> 原始报文，out_frame[0..7] -> 原始报文 + CRC16（低字节在前，高字节在后）
	static void buildModbusFrame(const std::uint8_t in_frame[6], std::uint8_t out_frame[8]);

private:
	// 行程/速度等参数（从 YAML 读取后不再修改）
	int min_height_ = 0;           // 最小高度 (mm)
	int max_height_ = 150;         // 最大高度 (mm)
	int default_speed_ = 1000;      // 默认最大速度（RPM）
	int default_acc_time_ = 500;    // 默认加速时间（ms）
	int default_dec_time_ = 500;    // 默认减速时间（ms）

	// Modbus 连接参数（从 YAML 读取后不再修改）
	std::string device_ = "/dev/ttyCH341USB0"; // 串口设备路径
	int baudrate_ = 115200;                // 波特率
	char parity_ = 'N';                    // 奇偶校验: 'N'/'E'/'O'
	int stopbits_ = 1;                     // 停止位
	int slave_id_ = 1;                     // 从站地址
	bool log_frames_ = false;              // 是否打印发送/接收的 Modbus 报文
	bool config_loaded_ = false;           // 是否成功从 YAML 加载配置

	int fd_ = -1;                          // 串口文件描述符（termios）
};

} // namespace lifter_modbus

