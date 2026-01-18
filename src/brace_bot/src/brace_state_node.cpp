#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace brace_bot
{

enum class HighLevelState
{
  IDLE = 0,
  LOW = 1,
  PICK = 2,
  HIGH = 3,
};

struct ActionStep
{
  // 是否配置了电机/电缸动作
  bool has_motor = false;
  bool has_lifter = false;

  // 电机目标位置（角度，单位：deg）
  double motor1_deg = 0.0;
  double motor2_deg = 0.0;
  double motor3_deg = 0.0;
  double motor4_deg = 0.0;
  double motor5_deg = 0.0;
  double motor6_deg = 0.0;

  // 电缸目标位置（mm）
  double lifter_mm = 0.0;

  // 本 step 执行动作后，进入下一 step 之前的等待时间（秒）
  double delay_s = 0.0;
};

struct ExecutionContext
{
  bool active = false;
  std::size_t step_index = 0;
  bool step_actions_sent = false;
  rclcpp::Time step_start_time;
};

class BraceStateNode : public rclcpp::Node
{
public:
  BraceStateNode()
  : rclcpp::Node("brace_state_node")
  {
    RCLCPP_INFO(get_logger(), "BraceStateNode starting...");

    // 发布器：两电机 + 电缸
    motor_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/motor/position_cmd", 10);
    lifter_cmd_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "/lifter_cmd", 10);

    // 状态命令：0=idle, 1=lift, 2=down
    state_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/brace_bot/state_cmd", 10,
      std::bind(&BraceStateNode::stateCmdCallback, this, std::placeholders::_1));

    // 从 YAML 读取动作序列
    if (!loadSequencesFromYaml())
    {
      RCLCPP_WARN(get_logger(), "Failed to load sequences from YAML, using empty sequences");
    }

    current_state_ = HighLevelState::IDLE;
    exec_ctx_.active = false;

    // 50Hz 定时器，驱动状态机
    timer_ = create_wall_timer(
      20ms, std::bind(&BraceStateNode::timerCallback, this));
  }

private:
  bool loadSequencesFromYaml()
  {
    std::string pkg_share;
    try
    {
      pkg_share = ament_index_cpp::get_package_share_directory("brace_bot");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get package share directory: %s", e.what());
      return false;
    }

    const std::string yaml_path = pkg_share + "/config/brace_sequences.yaml";
    RCLCPP_INFO(get_logger(), "Loading sequences from: %s", yaml_path.c_str());

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load YAML file: %s", e.what());
      return false;
    }

    if (!root["states"])
    {
      RCLCPP_ERROR(get_logger(), "YAML file missing 'states' root key");
      return false;
    }

    YAML::Node states_node = root["states"];

    // 四个高层状态：idle / low / pick / high
    loadStateSequence(states_node, "idle", HighLevelState::IDLE);
    loadStateSequence(states_node, "low", HighLevelState::LOW);
    loadStateSequence(states_node, "pick", HighLevelState::PICK);
    loadStateSequence(states_node, "high", HighLevelState::HIGH);

    return true;
  }

  void loadStateSequence(const YAML::Node &states_node,
                         const std::string &state_name,
                         HighLevelState state)
  {
    if (!states_node[state_name])
    {
      RCLCPP_WARN(get_logger(), "No sequence defined for state '%s'", state_name.c_str());
      sequences_[state].clear();
      return;
    }

    const YAML::Node &seq_node = states_node[state_name];
    if (!seq_node.IsSequence())
    {
      RCLCPP_ERROR(get_logger(), "State '%s' should be a sequence", state_name.c_str());
      sequences_[state].clear();
      return;
    }

    std::vector<ActionStep> seq;
    for (std::size_t i = 0; i < seq_node.size(); ++i)
    {
      const YAML::Node &step_node = seq_node[i];
      ActionStep step;

      if (step_node["motor1"])
      {
        step.has_motor = true;
        step.motor1_deg = step_node["motor1"].as<double>();
      }
      if (step_node["motor2"])
      {
        step.has_motor = true;
        step.motor2_deg = step_node["motor2"].as<double>();
      }
      if (step_node["motor3"])
      {
        step.has_motor = true;
        step.motor3_deg = step_node["motor3"].as<double>();
      }
      if (step_node["motor4"])
      {
        step.has_motor = true;
        step.motor4_deg = step_node["motor4"].as<double>();
      }
      if (step_node["motor5"])
      {
        step.has_motor = true;
        step.motor5_deg = step_node["motor5"].as<double>();
      }
      if (step_node["motor6"])
      {
        step.has_motor = true;
        step.motor6_deg = step_node["motor6"].as<double>();
      }
      if (step_node["lifter"])
      {
        step.has_lifter = true;
        step.lifter_mm = step_node["lifter"].as<double>();
      }
      if (step_node["delay"])
      {
        step.delay_s = step_node["delay"].as<double>();
      }

      seq.push_back(step);
    }

    sequences_[state] = seq;

    RCLCPP_INFO(get_logger(), "Loaded %zu steps for state '%s'",
                sequences_[state].size(), state_name.c_str());
  }

  void stateCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    HighLevelState new_state;
    switch (msg->data)
    {
      case 0: new_state = HighLevelState::IDLE; break;
      case 1: new_state = HighLevelState::LOW; break;
      case 2: new_state = HighLevelState::PICK; break;
      case 3: new_state = HighLevelState::HIGH; break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown state cmd: %d (0=IDLE,1=LOW,2=PICK,3=HIGH)", msg->data);
        return;
    }

    if (new_state == current_state_ && exec_ctx_.active)
    {
      RCLCPP_INFO(get_logger(), "State %d already active, ignore", msg->data);
      return;
    }

    startState(new_state);
  }

  void startState(HighLevelState state)
  {
    current_state_ = state;
    exec_ctx_ = ExecutionContext();

    auto it = sequences_.find(state);
    if (it == sequences_.end() || it->second.empty())
    {
      RCLCPP_INFO(get_logger(), "State has no sequence, stay idle");
      exec_ctx_.active = false;
      return;
    }

    exec_ctx_.active = true;
    exec_ctx_.step_index = 0;
    exec_ctx_.step_actions_sent = false;

    RCLCPP_INFO(get_logger(), "Start state %d with %zu steps",
                static_cast<int>(state), it->second.size());
  }

  void timerCallback()
  {
    if (!exec_ctx_.active)
    {
      return;
    }

    auto it = sequences_.find(current_state_);
    if (it == sequences_.end() || exec_ctx_.step_index >= it->second.size())
    {
      RCLCPP_INFO(get_logger(), "State sequence finished, back to IDLE");
      current_state_ = HighLevelState::IDLE;
      exec_ctx_.active = false;
      return;
    }

    auto &seq = it->second;
    auto &step = seq[exec_ctx_.step_index];
    auto now = this->get_clock()->now();
    // 1. 若本 step 的动作尚未发送，则立即发送（电机和电缸在本 step 内同时触发）
    if (!exec_ctx_.step_actions_sent)
    {
      if (step.has_motor)
      {
        publishMotorTargets(
          step.motor1_deg,
          step.motor2_deg,
          step.motor3_deg,
          step.motor4_deg,
          step.motor5_deg,
          step.motor6_deg);
      }

      if (step.has_lifter)
      {
        publishLifterTarget(step.lifter_mm);
      }

      exec_ctx_.step_actions_sent = true;
      exec_ctx_.step_start_time = now;
      return;
    }

    // 2. 动作已发送，等待 delay_s 之后进入下一个 step
    const double elapsed = (now - exec_ctx_.step_start_time).seconds();
    if (elapsed >= step.delay_s)
    {
      exec_ctx_.step_index++;
      exec_ctx_.step_actions_sent = false;
    }
  }

  void publishMotorTargets(double motor1_deg,
                           double motor2_deg,
                           double motor3_deg,
                           double motor4_deg,
                           double motor5_deg,
                           double motor6_deg)
  {
    const double rad_per_deg = M_PI / 180.0;
    sensor_msgs::msg::JointState js;
    js.position.resize(6);
    js.position[0] = motor1_deg * rad_per_deg;
    js.position[1] = motor2_deg * rad_per_deg;
    js.position[2] = motor3_deg * rad_per_deg;
    js.position[3] = motor4_deg * rad_per_deg;
    js.position[4] = motor5_deg * rad_per_deg;
    js.position[5] = motor6_deg * rad_per_deg;
    motor_pub_->publish(js);
  }

  void publishLifterTarget(double lifter_mm)
  {
    std_msgs::msg::Int32MultiArray cmd;
    // cmd_type = 0 表示位置模式，数据[1] = 目标高度 mm
    cmd.data.resize(2);
    cmd.data[0] = 0;  // 位置模式
    cmd.data[1] = static_cast<int32_t>(std::round(lifter_mm));
    lifter_cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lifter_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  HighLevelState current_state_ {HighLevelState::IDLE};
  std::map<HighLevelState, std::vector<ActionStep>> sequences_;
  ExecutionContext exec_ctx_;
};

}  // namespace brace_bot

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<brace_bot::BraceStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
