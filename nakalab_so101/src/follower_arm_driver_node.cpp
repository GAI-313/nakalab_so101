#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nakalab_so101/FeetechBus.hpp"
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

class FollowerArmDriverNode : public rclcpp::Node
{
public:
  FollowerArmDriverNode()
  : Node("follower_arm_driver_node")
  {
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<int>("baudrate", 1000000);
    this->declare_parameter<int>("max_speed", 2048);

    auto device_port = this->get_parameter("device").as_string();
    auto baudrate = this->get_parameter("baudrate").as_int();
    max_speed_ = this->get_parameter("max_speed").as_int();

    if (device_port.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'device' must be set!");
      rclcpp::shutdown();
      return;
    }

    bus_ = std::make_unique<nakalab_so101::FeetechBus>(device_port, baudrate);
    if (!bus_->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to the arm on port %s", device_port.c_str());
      rclcpp::shutdown();
      return;
    }
    
    // Enable torque on all motors
    bus_->enable_torque(get_all_motor_ids(), true);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("follower/joint_states", 10);
    joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "follower/joint_commands", 10,
      std::bind(&FollowerArmDriverNode::joint_cmd_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), // 50Hz
      std::bind(&FollowerArmDriverNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Follower Arm Driver started on %s", device_port.c_str());
  }

  ~FollowerArmDriverNode()
  {
    if (bus_ && bus_->is_connected()) {
      bus_->enable_torque(get_all_motor_ids(), false);
      bus_->disconnect();
    }
  }

private:
  void joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!bus_->is_connected()) {
      return;
    }
    
    max_speed_ = this->get_parameter("max_speed").as_int();

    bool use_velocity = (msg->velocity.size() == msg->name.size());
    std::map<uint8_t, nakalab_so101::ServoState> goal_states;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto& name = msg->name[i];
      if (motor_ids_.count(name)) {
        uint8_t mid = motor_ids_.at(name);
        
        nakalab_so101::ServoState state;
        state.position = rad_to_pos(msg->position[i]);

        if (use_velocity) {
          // 1 rad/s ~= 651 steps/s (from Python driver)
          double vel = std::abs(msg->velocity[i] * 651.0);
          state.speed = std::min(32767, static_cast<int>(vel));
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
    
    for (const auto& [id, state] : states) {
      if (id_to_name_.count(id)) {
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
    int val = static_cast<int>(rad * 4096.0 / (2.0 * M_PI) + 2048.0);
    return std::max(0, std::min(4095, val));
  }
  
  std::vector<uint8_t> get_all_motor_ids()
  {
    std::vector<uint8_t> ids;
    for(const auto& pair : motor_ids_) {
        ids.push_back(pair.second);
    }
    return ids;
  }

  std::unique_ptr<nakalab_so101::FeetechBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int max_speed_;

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
  auto node = std::make_shared<FollowerArmDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
