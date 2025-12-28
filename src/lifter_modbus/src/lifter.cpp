#include "lifter_modbus/lifter.hpp"

#include <array>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <type_traits>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace lifter_modbus
{

namespace
{
	constexpr int kMaxAduLength = 256; // 足够容纳典型 Modbus RTU ADU

	// 计算 Modbus RTU CRC16，多项式 0xA001，初始值 0xFFFF
	std::uint16_t compute_crc16(const std::uint8_t *data, std::size_t length)
	{
		std::uint16_t crc = 0xFFFF;
		for (std::size_t i = 0; i < length; ++i)
		{
			crc ^= static_cast<std::uint16_t>(data[i]);
			for (int j = 0; j < 8; ++j)
			{
				if (crc & 0x0001)
				{
					crc >>= 1;
					crc ^= 0xA001;
				}
				else
				{
					crc >>= 1;
				}
			}
		}
		return crc;
	}
} // namespace

Lifter::Lifter()
{
	// 默认值已在成员声明处设置，这里尝试从 YAML 覆盖
	// 优先从安装后的 share 目录查找 lifter_config.yaml，失败时退回源码树中的相对路径
	std::string config_path;
	try
	{
		config_path = ament_index_cpp::get_package_share_directory("lifter_modbus") + "/config/lifter_config.yaml";
	}
	catch (const std::exception &)
	{
		config_path = "src/lifter_modbus/config/lifter_config.yaml";
	}

	std::ifstream ifs(config_path);
	if (!ifs.is_open())
	{
		std::cerr << "[Lifter] Config file '" << config_path
		          << "' not found, using built-in default parameters" << std::endl;
		config_loaded_ = false;
		return;
	}

	std::string line;
	while (std::getline(ifs, line))
	{
		// 去掉行首空白
		std::size_t start = 0;
		while (start < line.size() && std::isspace(static_cast<unsigned char>(line[start])))
		{
			++start;
		}
		if (start >= line.size() || line[start] == '#')
		{
			continue;
		}

		auto trim_trailing = [](std::string &s) {
			while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
			{
				s.pop_back();
			}
		};

		auto parse_double = [](const std::string &s, double &out) {
			try
			{
				out = std::stod(s);
			}
			catch (...)
			{
				// 保持原值
			}
		};

		auto handle_string = [&](const char *key, std::string &target) {
			const std::size_t key_len = std::strlen(key);
			std::size_t pos = line.find(key, start);
			if (pos == std::string::npos)
			{
				return;
			}
			pos += key_len;
			// 跳过冒号和空白
			while (pos < line.size() && (line[pos] == ':' || std::isspace(static_cast<unsigned char>(line[pos]))))
			{
				++pos;
			}
			std::size_t end = pos;
			while (end < line.size() && line[end] != '#' && line[end] != '\r')
			{
				++end;
			}
			std::string value_str = line.substr(pos, end - pos);
			trim_trailing(value_str);
			// 去掉首尾引号
			if (!value_str.empty() && (value_str.front() == '"' || value_str.front() == '\''))
			{
				value_str.erase(0, 1);
			}
			if (!value_str.empty() && (value_str.back() == '"' || value_str.back() == '\''))
			{
				value_str.pop_back();
			}
			if (!value_str.empty())
			{
				target = value_str;
			}
		};

		auto parse_int = [](const std::string &s, int &out) {
			try
			{
				out = std::stoi(s);
			}
			catch (...)
			{
				// 保持原值
			}
		};

		auto handle_key = [&](const char *key, auto &target) {
			const std::size_t key_len = std::strlen(key);
			std::size_t pos = line.find(key, start);
			if (pos == std::string::npos)
			{
				return;
			}
			pos += key_len;
			// 跳过冒号和空白
			while (pos < line.size() && (line[pos] == ':' || std::isspace(static_cast<unsigned char>(line[pos]))))
			{
				++pos;
			}
			std::string value_str = line.substr(pos);
			if constexpr (std::is_same_v<std::decay_t<decltype(target)>, double>)
			{
				parse_double(value_str, target);
			}
			else
			{
				parse_int(value_str, target);
			}
		};

		// Modbus 参数
		handle_string("device", device_);
		handle_key("baudrate", baudrate_);

		// parity 单独处理：从当前行中解析第一个非空白且非引号字符
		{
			const char *key = "parity";
			const std::size_t key_len = std::strlen(key);
			std::size_t pos = line.find(key, start);
			if (pos != std::string::npos)
			{
				pos += key_len;
				while (pos < line.size() && (line[pos] == ':' || std::isspace(static_cast<unsigned char>(line[pos]))))
				{
					++pos;
				}
				if (pos < line.size())
				{
					char c = line[pos];
					if (c == '"' || c == '\'')
					{
						++pos;
						if (pos < line.size())
							c = line[pos];
					}
					if (c == 'N' || c == 'E' || c == 'O')
					{
						parity_ = c;
					}
				}
			}
		}

		handle_key("stopbits", stopbits_);
		handle_key("slave_id", slave_id_);

			// 是否打印报文: log_frames (0/1 或 true/false)
			{
				const char *key = "log_frames";
				const std::size_t key_len = std::strlen(key);
				std::size_t pos = line.find(key, start);
				if (pos != std::string::npos)
				{
					pos += key_len;
					while (pos < line.size() && (line[pos] == ':' || std::isspace(static_cast<unsigned char>(line[pos]))))
					{
						++pos;
					}
					std::string value_str = line.substr(pos);
					try
					{
						int v = std::stoi(value_str);
						log_frames_ = (v != 0);
					}
					catch (...)
					{
						if (value_str == "true" || value_str == "True" || value_str == "TRUE")
							log_frames_ = true;
						else if (value_str == "false" || value_str == "False" || value_str == "FALSE")
							log_frames_ = false;
					}
				}
			}

		// 限位和控制参数（单位：mm）
		handle_key("min_height", min_height_);
		handle_key("max_height", max_height_);
		handle_key("default_speed", default_speed_);
		handle_key("default_acc_time", default_acc_time_);
		handle_key("default_dec_time", default_dec_time_);
	}

	config_loaded_ = true;
}

Lifter::~Lifter()
{
	// 在关闭串口前尝试发送失能命令，避免节点/程序退出后驱动保持使能状态
	// 如果串口未初始化或失能失败，则忽略错误，继续资源清理
	if (fd_ >= 0)
	{
		(void)disable();
	}

	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}
}

// 初始化 Modbus（RTU 串口，数据位固定为 8）
// 所有参数来自构造函数中从 lifter_config.yaml 读取的成员变量
bool Lifter::initModbus()
{
	// 如已有连接，先关闭
	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}

	// 提示当前参数来源以及关键配置
	std::cerr << "[Lifter] initModbus using "
	          << (config_loaded_ ? "YAML config" : "built-in default parameters")
	          << ": device=" << device_
	          << ", baudrate=" << baudrate_
	          << ", parity=" << parity_
	          << ", stopbits=" << stopbits_
	          << ", slave_id=" << slave_id_
	          << ", default_speed=" << default_speed_
	          << ", acc_time=" << default_acc_time_
	          << ", dec_time=" << default_dec_time_
	          << ", log_frames=" << (log_frames_ ? 1 : 0)
	          << std::endl;

	fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd_ < 0)
	{
		std::cerr << "[Lifter] open failed for device=" << device_
		          << ": errno=" << errno << " (" << std::strerror(errno) << ")" << std::endl;
		return false;
	}

	termios tio{};
	if (tcgetattr(fd_, &tio) != 0)
	{
		std::cerr << "[Lifter] tcgetattr failed: errno=" << errno << " (" << std::strerror(errno) << ")" << std::endl;
		::close(fd_);
		fd_ = -1;
		return false;
	}

	cfmakeraw(&tio);

	// 设置波特率
	speed_t speed;
	switch (baudrate_)
	{
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	default:
		// 不支持的波特率，尝试 115200
		std::cerr << "[Lifter] unsupported baudrate=" << baudrate_ << ", fallback to 115200" << std::endl;
		baudrate_ = 115200;
		speed = B115200;
		break;
	}
	cfsetispeed(&tio, speed);
	cfsetospeed(&tio, speed);

	// 数据位：固定 8 位
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;

	// 校验位
	switch (parity_)
	{
	case 'E':
		tio.c_cflag |= PARENB;
		tio.c_cflag &= ~PARODD;
		break;
	case 'O':
		tio.c_cflag |= PARENB;
		tio.c_cflag |= PARODD;
		break;
	case 'N':
	default:
		tio.c_cflag &= ~PARENB;
		break;
	}

	// 停止位
	if (stopbits_ == 2)
	{
		tio.c_cflag |= CSTOPB;
	}
	else
	{
		tio.c_cflag &= ~CSTOPB;
	}

	// 无硬件流控
	tio.c_cflag &= ~CRTSCTS;

	// 打开接收，忽略 modem 控制线
	tio.c_cflag |= (CLOCAL | CREAD);

	// 设置读取超时：VTIME 2 -> 0.2s，总线空闲则返回
	tio.c_cc[VMIN]  = 0;
	tio.c_cc[VTIME] = 2;

	if (tcsetattr(fd_, TCSANOW, &tio) != 0)
	{
		std::cerr << "[Lifter] tcsetattr failed: errno=" << errno << " (" << std::strerror(errno) << ")" << std::endl;
		::close(fd_);
		fd_ = -1;
		return false;
	}

	return true;
}

bool Lifter::enable()
{
	if (fd_ < 0)
	{
		return false;
	}

	// 使能命令：01 06 20 00 00 08 83 CC
	// 这里只构造前 6 字节，CRC 由 sendRawRequest 内部计算
	const std::uint8_t raw_req[] = {
		static_cast<std::uint8_t>(slave_id_),
		0x06, 0x20, 0x00, 0x00, 0x08};
	if (!sendRawRequest(raw_req, static_cast<int>(sizeof(raw_req))))
	{
		return false;
	}

	std::uint8_t rsp[kMaxAduLength];
	int rc = receiveResponse(rsp, kMaxAduLength);
	return (rc > 0);
}

bool Lifter::disable()
{
	if (fd_ < 0)
	{
		return false;
	}

	// 失能命令：01 06 20 00 00 03 C2 0B
	// 同样只发送前 6 字节，CRC 由 sendRawRequest 生成
	const std::uint8_t raw_req[] = {
		static_cast<std::uint8_t>(slave_id_),
		0x06, 0x20, 0x00, 0x00, 0x03};
	if (!sendRawRequest(raw_req, static_cast<int>(sizeof(raw_req))))
	{
		return false;
	}

	std::uint8_t rsp[kMaxAduLength];
	int rc = receiveResponse(rsp, kMaxAduLength);
	return (rc > 0);
}

bool Lifter::setZero()
{
	if (fd_ < 0)
	{
		return false;
	}

	// 设置零位命令：01 06 20 02 00 01 E2 0A
	// 同样只发送前 6 字节，CRC 由 sendRawRequest 生成
	const std::uint8_t raw_req[] = {
		static_cast<std::uint8_t>(slave_id_),
		0x06, 0x20, 0x02, 0x00, 0x01};
	if (!sendRawRequest(raw_req, static_cast<int>(sizeof(raw_req))))
	{
		return false;
	}

	std::uint8_t rsp[kMaxAduLength];
	int rc = receiveResponse(rsp, kMaxAduLength);
	return (rc > 0);
}

bool Lifter::moveToPosition(double target_position)
{
	if (fd_ < 0)
	{
		return false;
	}
	// 目标位置单位为 mm，将其限制在 [min_height_, max_height_] 范围内
	int target_mm = static_cast<int>(target_position);
	int min_mm = min_height_;
	int max_mm = max_height_;
	if (target_mm < min_mm || target_mm > max_mm)
	{
		std::cerr << "[Lifter] moveToPosition: target_mm=" << target_mm
		          << " out of range [" << min_mm << ", " << max_mm << "]" << std::endl;
		return false;
	}

	// 线性换算关系：0 mm -> 0 cnt，95 mm -> 10,000,000 cnt
	// 先计算 double，再四舍五入为 32 位有符号整型
	double scale = 10000000.0 / 95.0;
	std::int32_t target_cnt = static_cast<std::int32_t>(
	    std::llround(static_cast<double>(target_mm) * scale));
	std::uint16_t pos_lo = static_cast<std::uint16_t>(target_cnt & 0xFFFF);
	std::uint16_t pos_hi = static_cast<std::uint16_t>((static_cast<std::uint32_t>(target_cnt) >> 16) & 0xFFFF);

	auto write_register = [&](std::uint16_t addr, std::uint16_t value) -> bool {
		std::uint8_t raw_req[6];
		raw_req[0] = static_cast<std::uint8_t>(slave_id_);
		raw_req[1] = 0x06; // 写单个保持寄存器
		raw_req[2] = static_cast<std::uint8_t>((addr >> 8) & 0xFF);
		raw_req[3] = static_cast<std::uint8_t>(addr & 0xFF);
		raw_req[4] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
		raw_req[5] = static_cast<std::uint8_t>(value & 0xFF);

		if (!sendRawRequest(raw_req, 6))
		{
			return false;
		}
		std::uint8_t rsp[kMaxAduLength];
		int rc = receiveResponse(rsp, kMaxAduLength);
		return (rc > 0);
	};

	// 1) 设置为位置模式：寄存器 0x0A1E = 0x0003 （你给的报文：01 06 0A 1E 00 03 AA 15）
	if (!write_register(0x0A1E, 0x0003))
	{
		return false;
	}

	// 2) 位置指令来源于内部指令：寄存器 0x0A02 = 0x0000（报文：01 06 0A 02 00 00 2B D2）
	if (!write_register(0x0A02, 0x0000))
	{
		return false;
	}

	// 3) 设置位置模式最大速度，例如 1000 RPM -> 0x03E8
	// 使用配置中的 default_speed_，限制在 0~0xFFFF 范围内
	int spd = default_speed_;
	if (spd < 0)
		spd = 0;
	else if (spd > 0xFFFF)
		spd = 0xFFFF;
	std::uint16_t speed_word = static_cast<std::uint16_t>(spd);
	// 寄存器 0x0915 = speed_word（报文示例：01 06 09 15 03 E8 9B 2C）
	if (!write_register(0x0915, speed_word))
	{
		return false;
	}

	// 4) 设置加速时间，单位 ms（例如 100ms -> 0x0064），寄存器 0x010E
	int acc = default_acc_time_;
	if (acc < 0)
		acc = 0;
	else if (acc > 0xFFFF)
		acc = 0xFFFF;
	std::uint16_t acc_word = static_cast<std::uint16_t>(acc);
	// 报文示例：01 06 01 0E 00 64 E8 1E
	if (!write_register(0x010E, acc_word))
	{
		return false;
	}

	// 5) 设置减速时间，单位 ms，寄存器 0x010F（报文：01 06 01 0F 00 64 B9 DE）
	int dec = default_dec_time_;
	if (dec < 0)
		dec = 0;
	else if (dec > 0xFFFF)
		dec = 0xFFFF;
	std::uint16_t dec_word = static_cast<std::uint16_t>(dec);
	if (!write_register(0x010F, dec_word))
	{
		return false;
	}

	// 6) 设置位置指令（单位：cnt），32 位，低 16 位寄存器 0x0916，高 16 位寄存器 0x0917
	// 示例 1,000,000 cnt = 0x0F4240：先发低 16 位 0x4240，再发高 16 位 0x000F
	// 低 16 位：报文 01 06 09 16 42 40 5A C2
	if (!write_register(0x0916, pos_lo))
	{
		return false;
	}
	// 高 16 位：报文 01 06 09 17 00 0F 7A 56
	if (!write_register(0x0917, pos_hi))
	{
		return false;
	}

	// 7) 执行位置指令：寄存器 0x0919 = 0x0001（报文：01 06 09 19 00 01 9A 51）
	if (!write_register(0x0919, 0x0001))
	{
		return false;
	}

	return true;
}

std::int32_t Lifter::getCurrentPosition() const
{
	if (fd_ < 0)
	{
		return 0;
	}

	// 读取位置命令：01 03 10 18 00 02 40 CC
	// 发送前 6 字节（地址、功能码、起始地址、高低字节、数量），CRC 由 sendRawRequest 生成
	const std::uint8_t raw_req[] = {
		static_cast<std::uint8_t>(slave_id_),
		0x03, 0x10, 0x18, 0x00, 0x02};
	if (!sendRawRequest(raw_req, static_cast<int>(sizeof(raw_req))))
	{
		return 0;
	}

	std::uint8_t rsp[kMaxAduLength];
	int rc = receiveResponse(rsp, kMaxAduLength);
	if (rc < 7)
	{
		// 至少需要: 地址(1) + 功能码(1) + 字节数(1) + 数据(4) + CRC(2)
		return 0;
	}

	// 简单校验从站地址和功能码（0x03 = 读保持寄存器）
	if (rsp[0] != static_cast<std::uint8_t>(slave_id_) || rsp[1] != 0x03)
	{
		return 0;
	}

	std::uint8_t byte_count = rsp[2];
	if (byte_count != 4)
	{
		return 0;
	}

	// 数据区 4 字节，格式为：LL LL HH HH（先低 16 位寄存器，再高 16 位）
	const std::uint8_t *data = &rsp[3];
	std::uint16_t reg_lo = static_cast<std::uint16_t>((data[0] << 8) | data[1]);
	std::uint16_t reg_hi = static_cast<std::uint16_t>((data[2] << 8) | data[3]);
	std::int32_t raw_pos = (static_cast<std::int32_t>(reg_hi) << 16) | reg_lo;

	// 将电机计数换算为高度（mm），0 mm -> 0 cnt, 95 mm -> 10,000,000 cnt
	double mm = static_cast<double>(raw_pos) * 95.0 / 10000000.0;
	std::int32_t height_mm = static_cast<std::int32_t>(std::llround(mm));
	return height_mm;
}

std::uint16_t Lifter::getDriveStatus() const
{
	// TODO: 通过 Modbus 读取驱动器状态寄存器（例如状态字或错误码）
	return 0u;
}

bool Lifter::abortPositionCommand()
{
	if (fd_ < 0)
	{
		return false;
	}

	// 将执行寄存器 0x0919 写为 0x0000：报文 01 06 09 19 00 00 5B 91
	std::uint8_t raw_req[6];
	raw_req[0] = static_cast<std::uint8_t>(slave_id_);
	raw_req[1] = 0x06; // 写单个保持寄存器
	raw_req[2] = 0x09;
	raw_req[3] = 0x19;
	raw_req[4] = 0x00;
	raw_req[5] = 0x00;

	if (!sendRawRequest(raw_req, 6))
	{
		return false;
	}

	std::uint8_t rsp[kMaxAduLength];
	int rc = receiveResponse(rsp, kMaxAduLength);
	return (rc > 0);
}

bool Lifter::sendRawRequest(const std::uint8_t *frame, int length) const
{
	if (fd_ < 0 || frame == nullptr || length <= 0)
	{
		return false;
	}

	if (log_frames_)
	{
		// 打印发送的 PDU（不含 CRC）
		std::ios old_state(nullptr);
		old_state.copyfmt(std::cerr);
		std::cerr << "[Lifter] TX PDU (" << length << "):";
		for (int i = 0; i < length; ++i)
		{
			std::cerr << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
			          << static_cast<int>(frame[i]);
		}
		std::cerr << std::endl;

		// 计算并打印实际在线路上的 RTU 帧（附带 CRC，低字节在前，高字节在后）
		std::uint16_t crc = compute_crc16(frame, static_cast<std::size_t>(length));
		std::uint8_t crc_lo = static_cast<std::uint8_t>(crc & 0xFF);
		std::uint8_t crc_hi = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
		std::cerr << "[Lifter] TX RTU (" << (length + 2) << "):";
		for (int i = 0; i < length; ++i)
		{
			std::cerr << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
			          << static_cast<int>(frame[i]);
		}
		std::cerr << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
		          << static_cast<int>(crc_lo);
		std::cerr << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
		          << static_cast<int>(crc_hi);
		std::cerr << std::endl;
		std::cerr.copyfmt(old_state);
	}

	// 组装完整 RTU 帧（带 CRC）并通过串口发送
	std::uint8_t adu[256];
	if (length + 2 > static_cast<int>(sizeof(adu)))
	{
		return false;
	}
	buildModbusFrame(frame, adu);

	int total_written = 0;
	int to_write = length + 2;
	while (total_written < to_write)
	{
		int rc = ::write(fd_, adu + total_written, static_cast<size_t>(to_write - total_written));
		if (rc < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			std::cerr << "[Lifter] write failed: errno=" << errno << " (" << std::strerror(errno) << ")" << std::endl;
			return false;
		}
		if (rc == 0)
		{
			break;
		}
		total_written += rc;
	}

	return (total_written == to_write);
}

int Lifter::receiveResponse(std::uint8_t *buffer, int max_length) const
{
	if (fd_ < 0 || buffer == nullptr || max_length <= 0)
	{
		return -1;
	}

	// 读取一帧：依赖串口超时返回当前已收到的数据，避免额外等待
	int rc = ::read(fd_, buffer, static_cast<size_t>(max_length));
	if (rc <= 0)
	{
		return -1;
	}

	if (log_frames_)
	{
		// 打印原始接收到的 ADU（通常为 RTU 帧，包含 CRC）
		std::ios old_state(nullptr);
		old_state.copyfmt(std::cerr);
		std::cerr << "[Lifter] RX (" << rc << "):";
		for (int i = 0; i < rc; ++i)
		{
			std::cerr << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
			          << static_cast<int>(buffer[i]);
		}
		std::cerr << std::endl;
		std::cerr.copyfmt(old_state);

		// 简单检测异常响应：功能码最高位为 1 表示异常
		if (rc >= 3 && (buffer[1] & 0x80) != 0)
		{
			std::uint8_t exc_code = buffer[2];
			std::cerr << "[Lifter] RX exception, code: 0x" << std::uppercase << std::hex
			          << std::setw(2) << std::setfill('0') << static_cast<int>(exc_code)
			          << std::dec << std::endl;
		}
	}

	return rc;
}

void Lifter::buildModbusFrame(const std::uint8_t in_frame[6], std::uint8_t out_frame[8])
{
	// 先拷贝 6 字节原始报文
	for (int i = 0; i < 6; ++i)
	{
		out_frame[i] = in_frame[i];
	}

	// 计算 CRC16
	std::uint16_t crc = compute_crc16(in_frame, 6);

	// 按低字节在前、高字节在后的顺序写入（符合标准 Modbus RTU）
	out_frame[6] = static_cast<std::uint8_t>(crc & 0xFF);        // 低字节
	out_frame[7] = static_cast<std::uint8_t>((crc >> 8) & 0xFF); // 高字节
}

} // namespace lifter_modbus

