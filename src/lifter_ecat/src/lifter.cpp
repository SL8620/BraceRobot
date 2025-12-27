#include "lifter_ecat/lifter.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <unistd.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

Lifter::Lifter(rclcpp::Node::SharedPtr node)
    : ethernet_interface_("eth0"),
      height_limits_({0, 1000}),
      speed_limits_({100}),
      stop_params_({500})
{
    // 初始化发布者和订阅者
    status_pub_ = node->create_publisher<std_msgs::msg::String>("/lifter_status", 10);
    height_pub_ = node->create_publisher<std_msgs::msg::Float64>("/lifter_height", 10);
    cmd_sub_ = node->create_subscription<std_msgs::msg::Float64>(
        "/lifter_cmd", 10, std::bind(&Lifter::cmd_callback, this, std::placeholders::_1));

    // 定期执行的回调
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Lifter::timer_callback, this)
    );
}

Lifter::~Lifter()
{
    shutdown();
}

void Lifter::shutdown()
{
    if (soem_initialized_) {
        ecx_close(&soem_context_);
        soem_initialized_ = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SOEM context closed");
    }
}

void Lifter::load_config()
{
    // 尝试按安装包位置读取配置：<package_share>/config/lifter_config.yaml
    std::string config_path;
    bool found = false;
    try {
        auto pkg_share = ament_index_cpp::get_package_share_directory("lifter_ecat");
        std::string candidate = pkg_share + "/config/lifter_config.yaml";
        std::ifstream f(candidate);
        if (f) { config_path = candidate; found = true; }
    } catch (const std::exception &e) {
        // ignore, we'll try other methods
    }

    if (!found) {
        // 尝试从 AMENT_PREFIX_PATH 中查找已安装包的 share 目录（有时 ament_index_cpp 不可用）
        const char *ament_prefix = std::getenv("AMENT_PREFIX_PATH");
        if (ament_prefix && *ament_prefix) {
            std::string prefixes(ament_prefix);
            size_t start = 0;
            while (start < prefixes.size()) {
                size_t colon = prefixes.find(':', start);
                std::string p = prefixes.substr(start, (colon==std::string::npos)?std::string::npos:colon-start);
                std::string candidate = p + "/share/lifter_ecat/config/lifter_config.yaml";
                std::ifstream f(candidate);
                if (f) { config_path = candidate; found = true; break; }
                if (colon==std::string::npos) break;
                start = colon + 1;
            }
        }
    }

    if (!found) {
        const char *env = std::getenv("LIFTER_CONFIG");
        if (env && *env) {
            config_path = std::string(env);
            found = true;
        }
    }

    if (!found) {
        config_path = std::string("/path/to/lifter_config.yaml");
    }

    std::ifstream config_file(config_path);
    if (config_file) {
        try {
            YAML::Node config = YAML::Load(config_file);
            if (config["ethernet_interface"]) ethernet_interface_ = config["ethernet_interface"].as<std::string>();
            if (config["height_limits"]) {
                height_limits_.min = config["height_limits"]["min"].as<int>();
                height_limits_.max = config["height_limits"]["max"].as<int>();
            }
            if (config["speed_limits"]) speed_limits_.max = config["speed_limits"]["max"].as<int>();
            if (config["stop_parameters"]) stop_params_.deceleration = config["stop_parameters"]["deceleration"].as<int>();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded config from %s", config_path.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse config file %s: %s", config_path.c_str(), e.what());
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open config file: %s", config_path.c_str());
    }
}

void Lifter::initialize_soem()
{
    // 初始化 SOEM 上下文
    memset(&soem_context_, 0, sizeof(soem_context_));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing EtherCAT on interface: %s",
                ethernet_interface_.c_str());

    // 1) 打开网卡
    int init_result = ecx_init(&soem_context_, ethernet_interface_.c_str());
    if (init_result <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "SOEM ecx_init failed! Result=%d. Possible reasons:\n"
                     "  - No EtherCAT hardware on %s\n"
                     "  - Insufficient permissions (need root or CAP_NET_RAW)\n"
                     "  - Wrong interface name",
                     init_result, ethernet_interface_.c_str());
        soem_initialized_ = false;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SOEM ecx_init succeeded");

    // 2) 扫描并配置从站
    int slave_count = ecx_config_init(&soem_context_);
    if (slave_count <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No slaves found by ecx_config_init");
        soem_initialized_ = false;
        ecx_close(&soem_context_);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EtherCAT slaves found: %d", slave_count);

    // 3) 配置 PDO 映射：使用一个本地 IOmap 缓冲区
    static uint8_t IOmap[4096];  // 足够覆盖我们 1 轴 Sine Drive 的 PDO
    memset(IOmap, 0, sizeof(IOmap));

    int used_bytes = ecx_config_map_group(&soem_context_, IOmap, 0);
    if (used_bytes <= 0) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "ecx_config_map_group returned %d, PDO mapping may have failed", used_bytes);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "PDO IOmap configured, used bytes: %d", used_bytes);
    }

    // 4) （可选）配置 DC
    ecx_configdc(&soem_context_);

    // 5) 切换到 SAFE_OP
    soem_context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&soem_context_, 0);
    ecx_statecheck(&soem_context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    uint16_t master_state = soem_context_.slavelist[0].state;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Master requested SAFE_OP, current state: 0x%04x", master_state);

    // 6) 检查每个从站的 PDO 缓冲区
    bool pdo_ok = true;
    for (int i = 1; i <= soem_context_.slavecount; ++i) {
        auto &sl = soem_context_.slavelist[i];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Slave %d '%s': state=0x%04x, Ibytes=%d, Obytes=%d",
                    i, sl.name, sl.state, sl.Ibytes, sl.Obytes);

        if (!sl.inputs || sl.Ibytes <= 0) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "  ✗ TxPDO (input) buffer not configured (inputs=%p, Ibytes=%d)",
                        sl.inputs, sl.Ibytes);
            pdo_ok = false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "  ✓ TxPDO (input) buffer at %p, %d bytes", sl.inputs, sl.Ibytes);
        }

        if (!sl.outputs || sl.Obytes <= 0) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "  ✗ RxPDO (output) buffer not configured (outputs=%p, Obytes=%d)",
                        sl.outputs, sl.Obytes);
            pdo_ok = false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "  ✓ RxPDO (output) buffer at %p, %d bytes", sl.outputs, sl.Obytes);
        }
    }

    if (!pdo_ok) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "PDO buffers are not fully configured. Motor APIs will degrade to cached values.");
    }

    // 7) 从 SAFE_OP 切换到 OPERATIONAL：
    //    - 按手册要求，主站需发送有效输出数据，然后请求状态转换
    if (pdo_ok) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Attempting Safe-Op → Operational transition (sending initial PDO data)");

        // 7.1 先发送几帧“安全”的输出数据（此处简单清零）
        for (int i = 1; i <= soem_context_.slavecount; ++i) {
            if (soem_context_.slavelist[i].outputs && soem_context_.slavelist[i].Obytes > 0) {
                memset(soem_context_.slavelist[i].outputs, 0, soem_context_.slavelist[i].Obytes);
            }
        }

        for (int k = 0; k < 10; ++k) {
            ecx_send_processdata(&soem_context_);
            ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);
            usleep(1000); // 1ms
        }

        // 7.2 请求所有从站进入 OPERATIONAL
        soem_context_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&soem_context_, 0);
        ecx_statecheck(&soem_context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

        uint16_t op_state = soem_context_.slavelist[0].state;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Master requested OPERATIONAL, current state: 0x%04x", op_state);

        for (int i = 1; i <= soem_context_.slavecount; ++i) {
            auto &sl = soem_context_.slavelist[i];
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Slave %d '%s' final AL state=0x%04x, ALstatuscode=0x%04x",
                        i, sl.name, sl.state, sl.ALstatuscode);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Skipping Safe-Op → Operational transition because PDO is not fully configured");
    }

    soem_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SOEM initialization finished");
}

void Lifter::cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double target_height = msg->data;
    if (target_height < height_limits_.min) {
        target_height = height_limits_.min;
    } else if (target_height > height_limits_.max) {
        target_height = height_limits_.max;
    }

    // 控制电机到达目标高度
    // (此处假设有控制代码)

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifter target height: %f", target_height);
}

void Lifter::timer_callback()
{
    uint32_t target_position = 0;
    uint32_t actual_position = 0;

    // 执行EtherCAT同步操作
    ecx_send_processdata(&soem_context_);
    ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);

    // 发布电机当前高度
    publish_height(static_cast<double>(actual_position));

    // 发布电机状态
    publish_status("Lifter is running");
}

void Lifter::publish_status(const std::string& status)
{
    std_msgs::msg::String status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);
}

void Lifter::publish_height(double height)
{
    std_msgs::msg::Float64 height_msg;
    height_msg.data = height;
    height_pub_->publish(height_msg);
}

void Lifter::dump_slave_info()
{
    if (!soem_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot dump slave info");
        return;
    }
    int count = soem_context_.slavecount;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dumping %d EtherCAT slaves:", count);
    for (int i = 1; i <= count; ++i) {
        // name is a fixed-size char array in SOEM structs
        const char *name = soem_context_.slavelist[i].name;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Slave %d: name=%s", i, name);
    }
}

void Lifter::dump_pdo_mapping()
{
    if (!soem_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot dump PDO mapping");
        return;
    }
    int count = soem_context_.slavecount;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PDO mapping (per slave):");
    for (int i = 1; i <= count; ++i) {
        // attempt to print number of PDOs found in EEPROM mapping if present
        // The ec_eepromPDOt structure is used by SOEM; we defensively check for presence
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Slave %d (%s):", i, soem_context_.slavelist[i].name);
        // Print SyncManager table sizes if available
        for (int sm = 0; sm < EC_MAXSM; ++sm) {
            uint16 smbits = soem_context_.slavelist[i].SM[sm].SMflags ? soem_context_.slavelist[i].SM[sm].SMflags : 0;
            // Only print defined SM entries (SM length > 0)
            if (soem_context_.slavelist[i].SM[sm].SMlength > 0) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  SM %d: StartAddr=%u Length=%u Flags=0x%04x",
                           sm,
                           soem_context_.slavelist[i].SM[sm].StartAddr,
                           soem_context_.slavelist[i].SM[sm].SMlength,
                           soem_context_.slavelist[i].SM[sm].SMflags);
            }
        }
    }
}

uint16 Lifter::read_slave_state(uint16 slave)
{
    // ════════════════════════════════════════════════════════════════
    // 读取指定从站的当前 EtherCAT 状态
    // 
    // 参数:
    //   slave  - 从站编号 (1 到 soem_context_.slavecount)
    //
    // 返回值:
    //   uint16 - 从站状态值，或 0 (如果错误)
    //   
    // 状态值定义 (SOEM ec_main.h):
    //   0x0001 = EC_STATE_INIT       (初始化中)
    //   0x0002 = EC_STATE_PREOP      (配置已完成) 
    //   0x0004 = EC_STATE_SAFEOP     (同步已完成)
    //   0x0008 = EC_STATE_OPERATIONAL (运行中)
    //   0x0010 = EC_STATE_ERROR      (故障)
    // ════════════════════════════════════════════════════════════════

    // 【步骤 1】检查 SOEM 是否已初始化
    // 只有在 ecx_init() 和 ecx_config_init() 成功完成后，
    // 从站状态信息才会被填充到 slavelist 中
    if (!soem_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot read slave state");
        return 0;  // 返回 0 表示初始化失败
    }

    // 【步骤 2】验证从站编号的有效性
    // soem_context_.slavecount 是在 ecx_config_init() 中设置的，
    // 表示网络中实际发现的从站总数
    // 
    // 例如：如果扫描到 1 个从站，则 slavecount = 1
    //       可以访问的从站号范围是：1 到 1
    // 如果请求编号 0 或编号 2，则无效
    if (slave < 1 || slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid slave number %u (valid range: 1-%d)", 
                     slave, soem_context_.slavecount);
        return 0;  // 返回 0 表示编号无效
    }

    // 【步骤 3】从内存中读取状态值
    // soem_context_.slavelist 是一个数组，包含所有从站的信息
    // 数组结构：
    //   slavelist[0] = 主站（通常不使用）
    //   slavelist[1] = 网络中的第 1 个从站 ← 本例读取的
    //   slavelist[2] = 网络中的第 2 个从站
    //   ...
    //   slavelist[127] = 最多支持 128 个从站
    //
    // 每个 ec_slavet 结构包含：
    //   .state          ← 我们要读的字段（16 位无符号整数）
    //   .ALstatuscode   ← 错误状态码
    //   .name           ← 从站名称（如 "Sine Drive"）
    //   .eep_man        ← 制造商 ID
    //   .eep_id         ← 产品型号
    //   ... (还有 80+ 个其他字段)
    //
    // 这个读取操作：
    // ① 纯粹的内存读取（不涉及网络通信）
    // ② 极快（< 1 微秒）
    // ③ 非阻塞（不会暂停执行）
    // ④ 获得的是缓存值（延迟约 10ms，因为定时器每 10ms 更新一次）
    uint16 state = soem_context_.slavelist[slave].state;

    // 【步骤 4】打印日志并返回
    // 日志格式: "Slave 1 (Sine Drive) current state: 0x0002"
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Slave %u (%s) current state: 0x%04x",
                slave, soem_context_.slavelist[slave].name, state);
    return state;
}

bool Lifter::set_slave_state(uint16 slave, uint16 target_state)
{
    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot set slave state");
        return false;
    }

    if (slave < 1 || slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid slave number %u (valid range: 1-%d)", 
                     slave, soem_context_.slavecount);
        return false;
    }

    uint16 current_state = soem_context_.slavelist[slave].state;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting slave %u (%s) from state 0x%04x to state 0x%04x",
                slave, soem_context_.slavelist[slave].name, current_state, target_state);

    /* Request the target state on slave; ecx_statecheck verifies it is reached */
    ecx_statecheck(&soem_context_, slave, target_state, EC_TIMEOUTSTATE);

    /* Check if state was successfully reached */
    current_state = soem_context_.slavelist[slave].state;
    uint16 alstatus = soem_context_.slavelist[slave].ALstatuscode;
    
    if (current_state == target_state) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Slave %u state transition successful (0x%04x)",
                    slave, current_state);
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                    "Slave %u state transition: requested 0x%04x, got 0x%04x instead. ALstatuscode: 0x%04x",
                    slave, target_state, current_state, alstatus);
        /* Don't treat this as an error; the slave might be transitioning through intermediate states */
        return false;
    }
}

bool Lifter::verify_slave_state(uint16 slave, uint16 expected_state)
{
    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized; cannot verify slave state");
        return false;
    }

    if (slave < 1 || slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid slave number %u (valid range: 1-%d)", 
                     slave, soem_context_.slavecount);
        return false;
    }

    uint16 current_state = soem_context_.slavelist[slave].state;
    uint16 alstatus = soem_context_.slavelist[slave].ALstatuscode;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Verifying slave %u (%s): expected=0x%04x, current=0x%04x, alstatus=0x%04x",
                slave, soem_context_.slavelist[slave].name, expected_state, current_state, alstatus);

    if (current_state == expected_state) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Slave %u state verification PASSED", slave);
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                    "Slave %u state verification FAILED: expected 0x%04x, got 0x%04x",
                    slave, expected_state, current_state);
        return false;
    }
}

double Lifter::read_motor_position()
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】读取电机的实际位置
    //
    // 使用 TxPDO (0x1A00) 中的：
    //   - 0x6064  Position Actual Value  (32-bit signed integer)
    //   - 通常单位：增量（counts）或脉冲
    //
    // 转换流程：
    //   原始值 (counts) -> 物理单位 (mm) -> 应用偏移
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                   "SOEM not initialized; cannot read motor position");
        return motor_position_offset_;  // 返回偏移量作为默认值
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Motor slave %u not found in network", slave);
        return -1.0;
    }

    // 【方法 1】直接从 PDO 输入缓冲区读取
    // TxPDO 结构（根据 0x1A00 映射）：
    // 
    //   offset 0:  uint16_t status_word      (0x6041)
    //   offset 2:  int32_t position_actual   (0x6064) <- 我们需要这个
    //   offset 6:  int32_t velocity_actual   (0x606C)
    //   offset 10: int8_t mode_display       (0x6061)
    //   offset 11: uint16_t error_code       (0x603F)
    //   offset 13: int16_t torque_actual     (0x6077)
    //   offset 15: uint32_t digital_inputs   (0x60FD)
    
    if (soem_context_.slavelist[slave].inputs != NULL && 
        soem_context_.slavelist[slave].Ibytes >= 6) {  // 需要至少 6 字节（status + position 的前 4 字节）
        
        // 从 offset 2 开始读取 32-bit 位置值
        int32_t raw_position = 0;
        uint8_t *input_ptr = soem_context_.slavelist[slave].inputs;
        
        // 跳过 status_word (2 bytes)，然后读取 position_actual (4 bytes)
        memcpy(&raw_position, input_ptr + 2, sizeof(int32_t));
        
        motor_actual_position_ = raw_position;
        
        // 转换公式：
        // 大多数 CiA402 伺服驱动器使用"增量/mm"的转换因子
        // 例如：4000 脉冲/mm 对于某些编码器
        // 这里假设 1000 脉冲/mm（可从硬件文档获取）
        double pulses_per_mm = 1000.0;  
        double physical_position = (double)raw_position / pulses_per_mm + motor_position_offset_;
        
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), 
                    "Motor position: raw=%d pulses, physical=%.2f mm (offset=%.2f mm)",
                    raw_position, physical_position, motor_position_offset_);
        
        return physical_position;
    } 
    
    // 【方法 2】备用：如果 PDO 缓冲区不可用，返回缓存值
    double cached_position = (double)motor_actual_position_ / 1000.0 + motor_position_offset_;
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
               "Motor PDO input not available; returning cached position: %.2f mm", 
               cached_position);
    
    return cached_position;
}

bool Lifter::write_motor_target_position(double position)
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】写入电机的目标位置
    //
    // 使用 RxPDO (0x1600) 中的：
    //   - 0x6040  Control Word             (16-bit)
    //   - 0x607A  Target Position          (32-bit signed integer)
    //   - 0x6081  Profile Velocity         (32-bit unsigned)
    //   - 0x6060  Modes of Operation       (8-bit)
    //
    // 流程：
    //   1. 检查位置范围
    //   2. 应用偏移计算实际位置
    //   3. 转换为脉冲数
    //   4. 写入 PDO 输出缓冲区（需要同时设置控制字和模式）
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "SOEM not initialized; cannot write motor target position");
        return false;
    }

    // 【检查范围】
    if (position < height_limits_.min || position > height_limits_.max) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Position %.2f mm out of valid range [%d, %d] mm",
                    position, height_limits_.min, height_limits_.max);
        return false;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Motor slave %u not found", slave);
        return false;
    }

    // 【转换单位】应用偏移并转换为脉冲
    double adjusted_position = position - motor_position_offset_;
    double pulses_per_mm = 1000.0;
    int32_t raw_target_position = (int32_t)(adjusted_position * pulses_per_mm);
    motor_target_position_ = raw_target_position;

    // 【写入 PDO 输出缓冲区】
    // RxPDO 结构（根据 0x1600 映射）：
    //
    //   offset 0:  uint16_t control_word      (0x6040)
    //   offset 2:  int32_t target_position    (0x607A) <- 写这里
    //   offset 6:  uint32_t profile_velocity  (0x6081)
    //   offset 10: int8_t modes_of_operation  (0x6060)
    //   offset 11: int16_t target_torque      (0x6071)
    //   offset 13: int32_t target_velocity    (0x60FF)
    //   offset 17: uint32_t acceleration      (0x60B2)
    
    if (soem_context_.slavelist[slave].outputs == NULL || 
        soem_context_.slavelist[slave].Obytes < 6) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Motor PDO output not configured (needs at least 6 bytes); cannot write");
        return false;
    }

    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    
    // 第一步：确保控制字启用了电机
    // 控制字位 0-3: shutdown(1), enable_op(1), quick_stop(1), enable_voltage(1) = 0x000F
    uint16_t control_word = 0x000F;  
    memcpy(output_ptr + 0, &control_word, sizeof(uint16_t));
    
    // 第二步：设置目标位置（offset 2, 4 bytes）
    memcpy(output_ptr + 2, &raw_target_position, sizeof(int32_t));
    
    // 第三步：设置工作模式为"轮廓位置模式"(MODE 1)
    // 这告诉驱动器使用轮廓位置控制（有速度和加速度规划）
    int8_t mode = (int8_t)cia402::MODE_PROFILED_POSITION;
    memcpy(output_ptr + 10, &mode, sizeof(int8_t));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
               "Motor target position updated: position=%.2f mm, raw=%d pulses, "
               "control_word=0x%04x, mode=%s",
               position, raw_target_position, control_word,
               cia402::mode_of_operation_to_string(mode));
    
    return true;
}

bool Lifter::set_motor_control_word(uint16_t control_word)
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】直接设置电机控制字
    //
    // 控制字格式（0x6040）：
    //   bit 0: Shutdown          (1=准备就绪, 0=禁用)
    //   bit 1: Enable operation  (1=使能, 0=禁用)
    //   bit 2: Quick stop        (1=允许, 0=快速停止)
    //   bit 3: Enable voltage    (1=使能, 0=禁用)
    //   bit 6: Fault reset       (脉冲清除故障)
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return false;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return false;
    }

    if (soem_context_.slavelist[slave].outputs == NULL) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO output not configured");
        return false;
    }

    // 写入控制字（offset 0, 2 bytes）
    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    memcpy(output_ptr + 0, &control_word, sizeof(uint16_t));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
               "Motor control word set to 0x%04x", control_word);
    
    return true;
}

cia402::StatusWord Lifter::read_motor_status_word()
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】读取电机状态字
    //
    // 状态字（0x6041）用于指示电机的当前状态和故障条件
    // 包含设备状态机 (DSM) 和各种标志位
    // ════════════════════════════════════════════════════════════════

    cia402::StatusWord status;
    status.value = 0;

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return status;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return status;
    }

    if (soem_context_.slavelist[slave].inputs == NULL || 
        soem_context_.slavelist[slave].Ibytes < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO input not configured");
        return status;
    }

    // 读取状态字（TxPDO offset 0, 2 bytes）
    uint8_t *input_ptr = soem_context_.slavelist[slave].inputs;
    memcpy(&status.value, input_ptr + 0, sizeof(uint16_t));
    
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), 
                "Motor status word: 0x%04x (%s)", 
                status.value, status.get_state_name());
    
    return status;
}

bool Lifter::set_motor_mode(cia402::ModeOfOperation mode)
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】设置电机工作模式
    //
    // 常用模式：
    //   1 = Profiled Position Mode (轮廓位置，推荐用于电缸)
    //   3 = Velocity Mode
    //   8 = Cyclic Synchronous Position Mode
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return false;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return false;
    }

    if (soem_context_.slavelist[slave].outputs == NULL || 
        soem_context_.slavelist[slave].Obytes < 11) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO output not configured");
        return false;
    }

    // 写入模式（RxPDO offset 10, 1 byte）
    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    int8_t mode_byte = (int8_t)mode;
    memcpy(output_ptr + 10, &mode_byte, sizeof(int8_t));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
               "Motor mode set to: %s (%d)", 
               cia402::mode_of_operation_to_string(mode), mode);
    
    return true;
}

bool Lifter::set_motor_velocity(int32_t velocity_rpm)
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】设置电机速度（profile velocity）
    //
    // 用于轮廓位置模式下的运动速度
    // 单位：通常是 RPM 或 mm/s（取决于驱动器）
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return false;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return false;
    }

    if (soem_context_.slavelist[slave].outputs == NULL || 
        soem_context_.slavelist[slave].Obytes < 10) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO output not configured");
        return false;
    }

    // 写入速度（RxPDO offset 6, 4 bytes）
    // 注意：这是 uint32，所以需要转换负数
    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    int32_t velocity_signed = velocity_rpm;
    memcpy(output_ptr + 6, &velocity_signed, sizeof(int32_t));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
               "Motor profile velocity set to: %d", velocity_rpm);
    
    return true;
}

bool Lifter::set_motor_acceleration(uint32_t acceleration)
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】设置电机加速度
    //
    // 用于轮廓位置模式下的加速度规划
    // 单位：取决于驱动器（通常是 rpm/s 或 mm/s²）
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return false;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return false;
    }

    if (soem_context_.slavelist[slave].outputs == NULL || 
        soem_context_.slavelist[slave].Obytes < 21) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO output not configured");
        return false;
    }

    // 写入加速度（RxPDO offset 17, 4 bytes）
    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    memcpy(output_ptr + 17, &acceleration, sizeof(uint32_t));
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
               "Motor acceleration set to: %u", acceleration);
    
    return true;
}

int32_t Lifter::read_motor_velocity()
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】读取电机实际速度
    // 
    // 从 TxPDO 中的 0x606C (Velocity Actual Value) 读取
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return 0;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return 0;
    }

    if (soem_context_.slavelist[slave].inputs == NULL || 
        soem_context_.slavelist[slave].Ibytes < 10) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO input not configured");
        return 0;
    }

    // 读取速度（TxPDO offset 6, 4 bytes）
    int32_t velocity = 0;
    uint8_t *input_ptr = soem_context_.slavelist[slave].inputs;
    memcpy(&velocity, input_ptr + 6, sizeof(int32_t));
    
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Motor actual velocity: %d", velocity);
    
    return velocity;
}

int16_t Lifter::read_motor_torque()
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】读取电机实际力矩
    // 
    // 从 TxPDO 中的 0x6077 (Torque Actual Value) 读取
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return 0;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return 0;
    }

    if (soem_context_.slavelist[slave].inputs == NULL || 
        soem_context_.slavelist[slave].Ibytes < 15) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO input not configured");
        return 0;
    }

    // 读取力矩（TxPDO offset 13, 2 bytes）
    int16_t torque = 0;
    uint8_t *input_ptr = soem_context_.slavelist[slave].inputs;
    memcpy(&torque, input_ptr + 13, sizeof(int16_t));
    
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Motor actual torque: %d", torque);
    
    return torque;
}

uint16_t Lifter::read_motor_error_code()
{
    // ════════════════════════════════════════════════════════════════
    // 【CiA402 标准】读取电机错误码
    // 
    // 从 TxPDO 中的 0x603F (Error Code) 读取
    // 常见错误码：
    //   0x0000 = No error
    //   0x1000 = Generic error
    //   0x2100 = Current too high
    //   0x2200 = Voltage problem
    // ════════════════════════════════════════════════════════════════

    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return 0xFFFF;
    }

    uint16 slave = 1;
    if (slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor slave %u not found", slave);
        return 0xFFFF;
    }

    if (soem_context_.slavelist[slave].inputs == NULL || 
        soem_context_.slavelist[slave].Ibytes < 13) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO input not configured");
        return 0xFFFF;
    }

    // 读取错误码（TxPDO offset 11, 2 bytes）
    uint16_t error_code = 0;
    uint8_t *input_ptr = soem_context_.slavelist[slave].inputs;
    memcpy(&error_code, input_ptr + 11, sizeof(uint16_t));
    
    if (error_code != 0) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                   "Motor error code: 0x%04x", error_code);
    }
    
    return error_code;
}

bool Lifter::enable_motor_cia402(uint16 slave, int timeout_ms_per_step)
{
    // 按照 CiA402 状态机标准流程，使能电机到 Operation Enabled
    if (!soem_initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SOEM not initialized");
        return false;
    }

    if (slave < 1 || slave > soem_context_.slavecount) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid slave number %u (valid range: 1-%d)",
                     slave, soem_context_.slavecount);
        return false;
    }

    // 确保 PDO 输出存在
    if (!soem_context_.slavelist[slave].outputs || soem_context_.slavelist[slave].Obytes < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor PDO output not configured");
        return false;
    }

    uint8_t *output_ptr = soem_context_.slavelist[slave].outputs;
    uint8_t *input_ptr  = soem_context_.slavelist[slave].inputs;

    auto send_recv = [this]() {
        ecx_send_processdata(&soem_context_);
        ecx_receive_processdata(&soem_context_, EC_TIMEOUTRET);
    };

    auto wait_state = [&](const char *label,
                          std::function<bool(const cia402::StatusWord &)> predicate) -> bool {
        const int step = 10; // 10ms 步进
        int elapsed = 0;
        while (elapsed < timeout_ms_per_step) {
            send_recv();
            cia402::StatusWord sw = read_motor_status_word();
            uint32_t din = 0;
            if (input_ptr && soem_context_.slavelist[slave].Ibytes >= 19) {
                memcpy(&din, input_ptr + 15, sizeof(uint32_t));
            }
            if (predicate(sw)) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                           "%s reached: 0x%04x (%s), DI=0x%08x",
                           label, sw.value, sw.get_state_name(), din);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(step));
            elapsed += step;
        }
        cia402::StatusWord sw = read_motor_status_word();
        uint32_t din = 0;
        if (input_ptr && soem_context_.slavelist[slave].Ibytes >= 19) {
            memcpy(&din, input_ptr + 15, sizeof(uint32_t));
        }
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Timeout waiting for %s, last status=0x%04x (%s), DI=0x%08x",
                    label, sw.value, sw.get_state_name(), din);
        return false;
    };

    // 在执行状态机前，先通过 PDO 设置工作模式为轮廓位置模式
    // 对应 RxPDO offset 10: Modes of Operation (0x6060)
    {
        int8_t mode = (int8_t)cia402::MODE_PROFILED_POSITION;
        memcpy(output_ptr + 10, &mode, sizeof(int8_t));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "CiA402 pre-step: set mode to Profiled Position (%d)", mode);
        // 发送几帧，让从站实际接收到模式设置
        for (int i = 0; i < 5; ++i) {
            send_recv();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 0) 若存在 Fault，则先 Fault Reset
    {
        send_recv();
        cia402::StatusWord sw = read_motor_status_word();
        if (sw.is_fault()) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Motor in FAULT state (0x%04x), sending fault reset", sw.value);
            uint16_t cw = 0x0080; // 常见 Fault Reset 控制字（具体可根据手册调整）
            memcpy(output_ptr + 0, &cw, sizeof(uint16_t));
            send_recv();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 1) Shutdown：0x0006 → Ready to Switch On
    {
        uint16_t cw = 0x0006;
        memcpy(output_ptr + 0, &cw, sizeof(uint16_t));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CiA402 step: Shutdown (0x0006)");
        if (!wait_state("Ready to Switch On", [](const cia402::StatusWord &sw) {
                return sw.is_ready_to_switch_on() && !sw.is_switched_on();
            })) {
            return false;
        }
    }

    // 2) Switch On：0x0007 → Switched On
    {
        uint16_t cw = 0x0007;
        memcpy(output_ptr + 0, &cw, sizeof(uint16_t));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CiA402 step: Switch On (0x0007)");
        if (!wait_state("Switched On", [](const cia402::StatusWord &sw) {
                return sw.is_ready_to_switch_on() && sw.is_switched_on() && !sw.is_operation_enabled();
            })) {
            return false;
        }
    }

    // 3) Enable Operation：0x000F → Operation Enabled
    {
        uint16_t cw = 0x000F;
        memcpy(output_ptr + 0, &cw, sizeof(uint16_t));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CiA402 step: Enable Operation (0x000F)");
        if (!wait_state("Operation Enabled", [](const cia402::StatusWord &sw) {
                return sw.is_operation_enabled();
            })) {
            return false;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CiA402 enable sequence completed successfully");
    return true;
}

