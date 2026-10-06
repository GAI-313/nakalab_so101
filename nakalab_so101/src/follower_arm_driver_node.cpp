#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nakalab_so101/Calibration.hpp"
#include "nakalab_so101/FeetechBus.hpp"
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <array>
#include <stdexcept>

namespace
{
std::string default_calibration_root()
{
  try {
    return ament_index_cpp::get_package_share_directory("nakalab_so101_description") +
           "/calibration";
  } catch (const std::exception &) {
    return "";
  }
}
}

class FollowerArmDriverNode : public rclcpp::Node
{
public:
  FollowerArmDriverNode()
  : Node("follower_arm_driver_node")
  {
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<int>("baudrate", 1000000);
    this->declare_parameter<int>("max_speed", 2048);
    this->declare_parameter<std::string>("calibration_file", "");
    this->declare_parameter<std::string>("calibration_root", default_calibration_root());
    this->declare_parameter<bool>("hold_current_position_on_startup", true);
    this->declare_parameter<bool>("read_only", true);

    auto device_port = this->get_parameter("device").as_string();
    auto baudrate = this->get_parameter("baudrate").as_int();
    max_speed_ = this->get_parameter("max_speed").as_int();
    read_only_ = this->get_parameter("read_only").as_bool();
    hold_current_position_on_startup_ =
      this->get_parameter("hold_current_position_on_startup").as_bool();

    load_calibration(
      this->get_parameter("calibration_file").as_string(),
      this->get_parameter("calibration_root").as_string());

    if (!read_only_ && (!calibration_loaded_ || !hold_current_position_on_startup_)) {
      throw std::runtime_error("制御には校正ファイルと起動時の現在位置保持が必要です．");
    }
    if (max_speed_ <= 0 || max_speed_ > 32767) {
      throw std::runtime_error("max_speed は 1〜32767 を指定してください．");
    }

    if (device_port.empty()) {
      throw std::runtime_error("Parameter 'device' must be set!");
    }

    bus_ = std::make_unique<nakalab_so101::FeetechBus>(device_port, baudrate);
    if (!bus_->connect()) {
      throw std::runtime_error(
              "Failed to connect to the arm on port " + device_port);
    }

    const auto motor_ids = get_all_motor_ids();
    verify_calibration();
    inspect_startup_state(motor_ids);
    if (!read_only_) {
      try {
        torque_control_active_ = true;
        hold_current_position_on_startup(motor_ids);
        for (const auto id : motor_ids) {
          if (!bus_->write_u8(id, 40, 1) || bus_->read_u8(id, 40) != 1) {
            throw std::runtime_error("トルク有効化の読み戻しに失敗しました．");
          }
        }
      } catch (...) {
        bus_->enable_torque(motor_ids, false);
        throw;
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "read_only=true: 目標位置・トルクは書き込まず，状態だけを配信します．");
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "follower/joint_states",
      10);
    if (!read_only_) {
      joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "follower/joint_commands", 10,
        std::bind(&FollowerArmDriverNode::joint_cmd_callback, this, std::placeholders::_1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), // 50Hz
      std::bind(&FollowerArmDriverNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Follower Arm Driver started on %s", device_port.c_str());
  }

  ~FollowerArmDriverNode()
  {
    if (bus_ && bus_->is_connected()) {
      if (torque_control_active_) {
        bus_->enable_torque(get_all_motor_ids(), false);
      }
      bus_->disconnect();
    }
  }

private:
  void joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (read_only_ || !bus_->is_connected()) {
      return;
    }

    max_speed_ = this->get_parameter("max_speed").as_int();
    if (max_speed_ <= 0 || max_speed_ > 32767) {
      RCLCPP_ERROR(get_logger(), "max_speed が範囲外のため指令を拒否します．");
      return;
    }

    bool use_velocity = (msg->velocity.size() == msg->name.size());
    std::map<uint8_t, nakalab_so101::ServoState> goal_states;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joint command with missing position values");
        return;
      }

      const auto & name = msg->name[i];
      if (motor_ids_.count(name)) {
        uint8_t mid = motor_ids_.at(name);

        const auto & limits = position_limits_.at(mid);
        if (!std::isfinite(msg->position[i]) ||
          msg->position[i] < pos_to_rad(limits.first) ||
          msg->position[i] > pos_to_rad(limits.second) ||
          (use_velocity && !std::isfinite(msg->velocity[i])))
        {
          RCLCPP_ERROR(get_logger(), "%s: 校正済み可動範囲外または不正な指令を拒否します．", name.c_str());
          return;
        }

        nakalab_so101::ServoState state;
        state.position = rad_to_pos(msg->position[i]);

        if (use_velocity) {
          // 1 rad/s ~= 651 steps/s (from Python driver)
          double vel = std::abs(msg->velocity[i] * 651.0);
          state.speed = static_cast<int>(std::min(32767.0, vel));
        } else {
          state.speed = max_speed_;
        }

        goal_states[mid] = state;
      }
    }

    if (!goal_states.empty()) {
      bus_->sync_write_goal_states(goal_states);
    }
  }

  void timer_callback()
  {
    if (!bus_->is_connected()) {
      return;
    }

    auto states = bus_->sync_read_present_states(get_all_motor_ids());

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();

    for (const auto & [id, state] : states) {
      if (id_to_name_.count(id)) {
        if (state.position < 0 || state.position > 4095) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "id=%u: 単回転範囲外の位置 %d を検出しました．状態配信を中止します．",
            id, state.position);
          return;
        }
        msg.name.push_back(id_to_name_.at(id));
        msg.position.push_back(pos_to_rad(state.position));

        // Speed conversion
        int16_t raw_speed = state.speed;
        double speed_rad_s = 0.0;
        if (raw_speed & 0x8000) { // Check sign bit
          speed_rad_s = -static_cast<double>(raw_speed & 0x7FFF) / 651.0;
        } else {
          speed_rad_s = static_cast<double>(raw_speed) / 651.0;
        }
        msg.velocity.push_back(speed_rad_s);

        // Load conversion
        int16_t raw_load = state.load;
        double effort = 0.0;
        if (raw_load & 0x0400) { // Check sign bit (bit 10)
          effort = -static_cast<double>(raw_load & 0x03FF) / 1000.0;
        } else {
          effort = static_cast<double>(raw_load & 0x03FF) / 1000.0;
        }
        msg.effort.push_back(effort);
      }
    }
    joint_state_pub_->publish(msg);
  }

  double pos_to_rad(int16_t pos)
  {
    return (static_cast<double>(pos) - 2048.0) * (2.0 * M_PI) / 4096.0;
  }

  int16_t rad_to_pos(double rad)
  {
    int val = static_cast<int>(std::lround(rad * 4096.0 / (2.0 * M_PI) + 2048.0));
    return std::max(0, std::min(4095, val));
  }

  void load_calibration(const std::string & calibration_file, const std::string & calibration_root)
  {
    if (!calibration_file.empty()) {
      calibration_ = nakalab_so101::Calibration::load_from_file(calibration_file);
      calibration_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Using calibration file: %s", calibration_file.c_str());
      return;
    }

    const auto candidates = nakalab_so101::Calibration::find_follower_files(calibration_root);
    if (candidates.size() == 1) {
      RCLCPP_WARN(
        this->get_logger(), "calibration_file is empty; automatically using %s",
        candidates.front().c_str());
      calibration_ = nakalab_so101::Calibration::load_from_file(candidates.front());
      calibration_loaded_ = true;
    } else if (candidates.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "No follower calibration JSON found; using uncorrected conversion");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Found %zu follower calibration JSON files; refusing automatic selection and using "
        "uncorrected conversion",
        candidates.size());
    }
  }

  void verify_calibration()
  {
    if (!calibration_loaded_) {
      return;
    }

    for (const auto & [joint_name, motor_id] : motor_ids_) {
      const auto motor_offset = bus_->read_homing_offset(motor_id);
      if (!motor_offset) {
        throw std::runtime_error(
                "Failed to read homing offset for joint '" + joint_name + "'");
      }

      const int expected_offset = calibration_.homing_offset(joint_name);
      if (*motor_offset != expected_offset) {
        throw std::runtime_error(
                "Homing offset mismatch for joint '" + joint_name + "': calibration file " +
                std::to_string(expected_offset) + ", motor " + std::to_string(*motor_offset));
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Verified calibration JSON against motor homing offsets; driver does not write calibration");
  }

  void inspect_startup_state(const std::vector<uint8_t> & motor_ids)
  {
    const auto states = bus_->sync_read_present_states(motor_ids);
    if (states.size() != motor_ids.size()) {
      throw std::runtime_error("起動時に全関節の状態を読み取れません．");
    }
    for (const auto id : motor_ids) {
      const auto minimum = bus_->read_u16(id, 9);
      const auto maximum = bus_->read_u16(id, 11);
      const auto mode = bus_->read_u8(id, 33);
      const auto phase = bus_->read_u8(id, 18);
      const auto torque = bus_->read_u8(id, 40);
      const auto goal = bus_->read_u16(id, 42);
      if (!minimum || !maximum || !mode || !phase || !torque || !goal) {
        throw std::runtime_error("起動時にサーボ設定を読み取れません．");
      }
      position_limits_[id] = {*minimum, *maximum};
      const auto & state = states.at(id);
      RCLCPP_INFO(
        get_logger(),
        "%s id=%u raw=%d rad=%.6f goal=%u limits=[%u,%u] torque=%u mode=%u phase=%u",
        id_to_name_.at(id).c_str(), id, state.position, pos_to_rad(state.position),
        *goal, *minimum, *maximum, *torque, *mode, *phase);
      if (read_only_) {
        continue;
      }
      const auto prefix = id_to_name_.at(id) + " id=" + std::to_string(id) + ": ";
      if (*minimum >= *maximum || *maximum > 4095) {
        throw std::runtime_error(
                prefix + "可動範囲の設定が不正です: limits=[" +
                std::to_string(*minimum) + "," + std::to_string(*maximum) + "]");
      }
      if (*mode != 0 || (*phase & 0x10) != 0) {
        throw std::runtime_error(
                prefix + "単回転の位置制御設定ではありません: mode=" +
                std::to_string(*mode) + " phase=" + std::to_string(*phase));
      }
      if (*torque != 0) {
        throw std::runtime_error(
                prefix + "トルクが既に有効なため設定を変更せず終了します: torque=" +
                std::to_string(*torque));
      }
      if (state.position < *minimum || state.position > *maximum) {
        const int excess = state.position < *minimum ?
          *minimum - state.position : state.position - *maximum;
        throw std::runtime_error(
                prefix + "現在位置が可動範囲外です: raw=" + std::to_string(state.position) +
                " limits=[" + std::to_string(*minimum) + "," + std::to_string(*maximum) +
                "] 超過=" + std::to_string(excess) + " ステップ (" +
                std::to_string(excess * 360.0 / 4096.0) +
                " 度)．範囲外の保持目標は送信せず，制御を開始しません．");
      }
    }
  }

  void hold_current_position_on_startup(const std::vector<uint8_t> & motor_ids)
  {
    const auto states = bus_->sync_read_present_states(motor_ids);
    if (states.size() != motor_ids.size()) {
      throw std::runtime_error("現在位置保持の前に全関節の状態を読み取れません．");
    }

    std::map<uint8_t, nakalab_so101::ServoState> hold_states;
    for (const auto & [motor_id, state] : states) {
      const auto & limits = position_limits_.at(motor_id);
      if (state.position < limits.first || state.position > limits.second || state.speed != 0) {
        throw std::runtime_error("現在位置が可動範囲外，または関節が動いているため制御を開始しません．");
      }
      hold_states[motor_id] = {state.position, max_speed_, 0};
    }

    if (!bus_->sync_write_goal_states(hold_states)) {
      throw std::runtime_error("Failed to synchronize current positions before enabling torque");
    }

    for (const auto & [motor_id, state] : hold_states) {
      const auto goal = bus_->read_u16(motor_id, 42);
      if (!goal || *goal != state.position) {
        throw std::runtime_error("目標位置の読み戻しが現在位置と一致しないため，トルクを有効にしません．");
      }
    }
    const auto confirmed_states = bus_->sync_read_present_states(motor_ids);
    for (const auto & [motor_id, state] : hold_states) {
      const auto actual = confirmed_states.find(motor_id);
      if (actual == confirmed_states.end() || actual->second.speed != 0 ||
        std::abs(actual->second.position - state.position) > 10)
      {
        throw std::runtime_error("起動準備中に姿勢が変わったため，トルクを有効にしません．");
      }
    }
    RCLCPP_INFO(
      this->get_logger(),
      "全関節の目標位置と現在位置を読み戻して確認しました．");
  }

  std::vector<uint8_t> get_all_motor_ids()
  {
    std::vector<uint8_t> ids;
    for (const auto & pair : motor_ids_) {
      ids.push_back(pair.second);
    }
    return ids;
  }

  std::unique_ptr<nakalab_so101::FeetechBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int max_speed_;
  bool hold_current_position_on_startup_{true};
  bool read_only_{true};
  bool torque_control_active_{false};
  std::map<uint8_t, std::pair<int, int>> position_limits_;
  nakalab_so101::Calibration calibration_;
  bool calibration_loaded_{false};

  const std::map<std::string, uint8_t> motor_ids_ = {
    {"shoulder_pan", 1},
    {"shoulder_lift", 2},
    {"elbow_flex", 3},
    {"wrist_flex", 4},
    {"wrist_roll", 5},
    {"gripper", 6}
  };
  const std::map<uint8_t, std::string> id_to_name_ = {
    {1, "shoulder_pan"},
    {2, "shoulder_lift"},
    {3, "elbow_flex"},
    {4, "wrist_flex"},
    {5, "wrist_roll"},
    {6, "gripper"}
  };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<FollowerArmDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("follower_arm_driver_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
