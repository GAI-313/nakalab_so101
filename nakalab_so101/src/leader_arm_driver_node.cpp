#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nakalab_so101/FeetechBus.hpp"
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <memory>

class LeaderArmDriverNode : public rclcpp::Node
{
public:
  LeaderArmDriverNode()
  : Node("leader_arm_driver_node")
  {
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<int>("baudrate", 1000000);
    this->declare_parameter<bool>("enable_teleop", false);

    auto device_port = this->get_parameter("device").as_string();
    auto baudrate = this->get_parameter("baudrate").as_int();
    teleop_enabled_ = this->get_parameter("enable_teleop").as_bool();

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
    
    // Ensure torque is disabled for leader arm
    bus_->enable_torque(get_all_motor_ids(), false);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("leader/joint_states", 10);
    teleop_service_ = this->create_service<std_srvs::srv::SetBool>(
      "teleop",
      std::bind(&LeaderArmDriverNode::teleop_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), // 50Hz
      std::bind(&LeaderArmDriverNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Leader Arm Driver started on %s", device_port.c_str());
  }

  ~LeaderArmDriverNode()
  {
    if (bus_ && bus_->is_connected()) {
      bus_->enable_torque(get_all_motor_ids(), false);
      bus_->disconnect();
    }
  }

private:
  void teleop_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    teleop_enabled_ = request->data;
    response->success = true;
    response->message = "Teleop enabled: " + std::to_string(teleop_enabled_);
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void timer_callback()
  {
    if (!teleop_enabled_ || !bus_->is_connected()) {
      return;
    }

    auto positions = bus_->sync_read_present_positions(get_all_motor_ids());
    
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();
    
    for (const auto& [id, pos] : positions) {
      if (id_to_name_.count(id)) {
        msg.name.push_back(id_to_name_.at(id));
        msg.position.push_back(pos_to_rad(pos));
      }
    }
    joint_state_pub_->publish(msg);
  }

  double pos_to_rad(int16_t pos)
  {
    // STS3215 pos is 0-4095, mapping to ~ 0-360 deg. 2048 is approx 0 rad.
    return (static_cast<double>(pos) - 2048.0) * (2.0 * M_PI) / 4096.0;
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
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool teleop_enabled_;

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
  auto node = std::make_shared<LeaderArmDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
