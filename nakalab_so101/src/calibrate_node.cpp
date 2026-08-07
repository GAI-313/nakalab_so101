#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nakalab_so101/FeetechBus.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr int kHalfTurnPosition = 2047;
constexpr uint8_t kRegMinPositionLimit = 9;
constexpr uint8_t kRegMaxPositionLimit = 11;
constexpr uint8_t kRegPhase = 18;
constexpr uint8_t kRegOperatingMode = 33;
constexpr uint8_t kPositionMode = 0;
constexpr uint8_t kPhaseFeedbackBit = 0x10;

const std::vector<std::string> kValidArmTypes = {"so101_leader", "so101_follower"};
const std::map<std::string, uint8_t> kMotorIds = {
  {"shoulder_pan", 1},
  {"shoulder_lift", 2},
  {"elbow_flex", 3},
  {"wrist_flex", 4},
  {"wrist_roll", 5},
  {"gripper", 6},
};

std::vector<uint8_t> motor_ids()
{
  std::vector<uint8_t> ids;
  for (const auto & [_, motor_id] : kMotorIds) {
    ids.push_back(motor_id);
  }
  return ids;
}

std::filesystem::path default_calibration_root()
{
  if (const char * pixi_root = std::getenv("PIXI_PROJECT_ROOT")) {
    const auto candidate =
      std::filesystem::path(pixi_root) / "nakalab_so101_description" / "calibration";
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }

  try {
    const auto package_share =
      ament_index_cpp::get_package_share_directory("nakalab_so101_description");
    return std::filesystem::path(package_share) / "calibration";
  } catch (const std::exception &) {
    return std::filesystem::path("nakalab_so101_description") / "calibration";
  }
}

bool is_file_stem(const std::string & value)
{
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  const auto path = std::filesystem::path(value);
  return path.filename() == value && value.find('/') == std::string::npos;
}
}

struct MotorCalibration
{
  uint8_t motor_id;
  int drive_mode;
  int homing_offset;
  int range_min;
  int range_max;
};

class CalibrateNode : public rclcpp::Node
{
public:
  CalibrateNode()
  : Node("calibrate")
  {
    declare_parameter<std::string>("device", "");
    declare_parameter<std::string>("arm_type", "");
    declare_parameter<std::string>("calibration_id", "");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<bool>("overwrite", false);
    declare_parameter<std::string>("calibration_root", default_calibration_root().string());
  }

  void run()
  {
    const auto device = get_parameter("device").as_string();
    const auto arm_type = get_parameter("arm_type").as_string();
    const auto calibration_id = get_parameter("calibration_id").as_string();
    const auto baudrate = get_parameter("baudrate").as_int();
    const auto overwrite = get_parameter("overwrite").as_bool();
    const auto calibration_root = get_parameter("calibration_root").as_string();

    validate_config(device, arm_type, calibration_id);
    const auto output_path =
      std::filesystem::path(calibration_root) / arm_type / (calibration_id + ".json");
    confirm_overwrite(output_path, overwrite);

    nakalab_so101::FeetechBus bus(device, baudrate);
    if (!bus.connect()) {
      throw std::runtime_error("Failed to connect to the arm on port " + device);
    }

    bool locked = false;
    const auto ids = motor_ids();
    try {
      verify_motor_ids(bus, ids);
      prepare_motors(bus, ids);

      wait_for_enter(
        "Move the arm to the middle of its range of motion, then press Enter.");
      const auto center_positions = read_required_positions(bus);
      const auto homing_offsets = compute_homing_offsets(center_positions);
      write_homing_offsets(bus, homing_offsets);

      std::cout << "Move each joint through its full range of motion. "
                << "Press Enter when finished." << std::endl;
      const auto ranges = record_ranges_of_motion(bus);

      const auto calibration = build_calibration(homing_offsets, ranges);
      write_range_limits(bus, calibration);
      bus.lock_eeprom(ids);
      locked = true;
      verify_calibration(bus, calibration);
      save_calibration(output_path, calibration);
      RCLCPP_INFO(get_logger(), "Saved calibration to %s", output_path.c_str());
    } catch (...) {
      if (!locked) {
        bus.lock_eeprom(ids);
      }
      bus.disconnect();
      throw;
    }

    bus.disconnect();
  }

private:
  void validate_config(
    const std::string & device,
    const std::string & arm_type,
    const std::string & calibration_id)
  {
    if (device.empty()) {
      throw std::runtime_error("Parameter 'device' must be set.");
    }
    if (std::find(kValidArmTypes.begin(), kValidArmTypes.end(), arm_type) ==
      kValidArmTypes.end())
    {
      throw std::runtime_error("Parameter 'arm_type' must be one of: so101_leader, so101_follower");
    }
    if (!is_file_stem(calibration_id)) {
      throw std::runtime_error("Parameter 'calibration_id' must be a file stem.");
    }
  }

  void confirm_overwrite(const std::filesystem::path & output_path, bool overwrite)
  {
    if (overwrite || !std::filesystem::exists(output_path)) {
      return;
    }

    std::cout << "Calibration file exists at " << output_path.string()
              << ". Overwrite? [y/N] ";
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "y" && answer != "yes") {
      throw std::runtime_error("Calibration cancelled to avoid overwrite.");
    }
  }

  void wait_for_enter(const std::string & prompt)
  {
    std::cout << prompt << std::endl;
    std::string line;
    std::getline(std::cin, line);
  }

  void verify_motor_ids(
    nakalab_so101::FeetechBus & bus,
    const std::vector<uint8_t> & ids)
  {
    const auto positions = bus.sync_read_present_positions(ids);
    std::vector<int> missing;
    for (const auto motor_id : ids) {
      if (!positions.count(motor_id)) {
        missing.push_back(motor_id);
      }
    }
    if (!missing.empty()) {
      std::string message = "Missing motor IDs:";
      for (const auto motor_id : missing) {
        message += " " + std::to_string(motor_id);
      }
      throw std::runtime_error(message);
    }
  }

  void prepare_motors(
    nakalab_so101::FeetechBus & bus,
    const std::vector<uint8_t> & ids)
  {
    bus.enable_torque(ids, false);
    bus.unlock_eeprom(ids);

    for (const auto motor_id : ids) {
      require(bus.write_u8(motor_id, kRegOperatingMode, kPositionMode), motor_id, "write operating mode");
      const auto phase = bus.read_u8(motor_id, kRegPhase);
      if (phase && (*phase & kPhaseFeedbackBit)) {
        require(
          bus.write_u8(motor_id, kRegPhase, static_cast<uint8_t>(*phase & ~kPhaseFeedbackBit)),
          motor_id, "clear phase feedback bit");
      }
    }
  }

  std::map<std::string, int> read_required_positions(nakalab_so101::FeetechBus & bus)
  {
    const auto positions_by_id = bus.sync_read_present_positions(motor_ids());
    std::map<std::string, int> positions;
    std::vector<std::string> missing;
    for (const auto & [name, motor_id] : kMotorIds) {
      const auto position = positions_by_id.find(motor_id);
      if (position == positions_by_id.end()) {
        missing.push_back(name);
      } else {
        positions[name] = position->second;
      }
    }
    if (!missing.empty()) {
      std::string message = "Missing position reads for joints:";
      for (const auto & name : missing) {
        message += " " + name;
      }
      throw std::runtime_error(message);
    }
    return positions;
  }

  std::map<std::string, int> compute_homing_offsets(
    const std::map<std::string, int> & center_positions)
  {
    std::map<std::string, int> homing_offsets;
    for (const auto & [name, position] : center_positions) {
      homing_offsets[name] = position - kHalfTurnPosition;
    }
    return homing_offsets;
  }

  void write_homing_offsets(
    nakalab_so101::FeetechBus & bus,
    const std::map<std::string, int> & homing_offsets)
  {
    for (const auto & [name, homing_offset] : homing_offsets) {
      const auto motor_id = kMotorIds.at(name);
      require(bus.write_homing_offset(motor_id, homing_offset), motor_id, "write homing offset");
    }
  }

  std::map<std::string, std::map<std::string, int>> record_ranges_of_motion(
    nakalab_so101::FeetechBus & bus)
  {
    std::map<std::string, std::map<std::string, int>> ranges;
    bool initialized = false;
    std::atomic<bool> stop{false};
    std::thread input_thread([&stop]() {
      std::string line;
      std::getline(std::cin, line);
      stop = true;
    });

    auto last_display = std::chrono::steady_clock::now();
    while (!stop) {
      const auto positions = read_required_positions(bus);
      for (const auto & [name, position] : positions) {
        if (!initialized || !ranges.count(name)) {
          ranges[name] = {{"min", position}, {"max", position}};
        }
        ranges[name]["min"] = std::min(ranges[name]["min"], position);
        ranges[name]["max"] = std::max(ranges[name]["max"], position);
      }
      initialized = true;

      const auto now = std::chrono::steady_clock::now();
      if (now - last_display >= std::chrono::milliseconds(250)) {
        print_range_table(positions, ranges);
        last_display = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    input_thread.join();
    return ranges;
  }

  void print_range_table(
    const std::map<std::string, int> & positions,
    const std::map<std::string, std::map<std::string, int>> & ranges)
  {
    std::cout << "joint             MIN | POS | MAX" << std::endl;
    for (const auto & [name, _] : kMotorIds) {
      std::cout << name;
      for (size_t i = name.size(); i < 16; ++i) {
        std::cout << ' ';
      }
      std::cout << ' ' << ranges.at(name).at("min")
                << " | " << positions.at(name)
                << " | " << ranges.at(name).at("max") << std::endl;
    }
  }

  std::map<std::string, MotorCalibration> build_calibration(
    const std::map<std::string, int> & homing_offsets,
    const std::map<std::string, std::map<std::string, int>> & ranges)
  {
    std::map<std::string, MotorCalibration> calibration;
    for (const auto & [name, motor_id] : kMotorIds) {
      calibration[name] = {
        motor_id,
        0,
        homing_offsets.at(name),
        ranges.at(name).at("min"),
        ranges.at(name).at("max"),
      };
    }
    return calibration;
  }

  void write_range_limits(
    nakalab_so101::FeetechBus & bus,
    const std::map<std::string, MotorCalibration> & calibration)
  {
    for (const auto & [_, values] : calibration) {
      require(
        bus.write_u16(values.motor_id, kRegMinPositionLimit, static_cast<uint16_t>(values.range_min)),
        values.motor_id, "write range min");
      require(
        bus.write_u16(values.motor_id, kRegMaxPositionLimit, static_cast<uint16_t>(values.range_max)),
        values.motor_id, "write range max");
    }
  }

  void verify_calibration(
    nakalab_so101::FeetechBus & bus,
    const std::map<std::string, MotorCalibration> & calibration)
  {
    for (const auto & [name, values] : calibration) {
      const auto homing_offset = bus.read_homing_offset(values.motor_id);
      if (!homing_offset || *homing_offset != values.homing_offset) {
        throw std::runtime_error(name + ": homing offset verification failed.");
      }

      const auto range_min = bus.read_u16(values.motor_id, kRegMinPositionLimit);
      const auto range_max = bus.read_u16(values.motor_id, kRegMaxPositionLimit);
      if (!range_min || !range_max ||
        *range_min != static_cast<uint16_t>(values.range_min) ||
        *range_max != static_cast<uint16_t>(values.range_max))
      {
        throw std::runtime_error(name + ": range verification failed.");
      }
    }
  }

  void save_calibration(
    const std::filesystem::path & output_path,
    const std::map<std::string, MotorCalibration> & calibration)
  {
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output.is_open()) {
      throw std::runtime_error("Unable to open calibration file: " + output_path.string());
    }

    output << "{\n";
    size_t index = 0;
    for (const auto & [name, values] : calibration) {
      output << "    \"" << name << "\": {\n"
             << "        \"id\": " << static_cast<int>(values.motor_id) << ",\n"
             << "        \"drive_mode\": " << values.drive_mode << ",\n"
             << "        \"homing_offset\": " << values.homing_offset << ",\n"
             << "        \"range_min\": " << values.range_min << ",\n"
             << "        \"range_max\": " << values.range_max << "\n"
             << "    }";
      ++index;
      output << (index == calibration.size() ? "\n" : ",\n");
    }
    output << "}\n";
  }

  void require(bool status, uint8_t motor_id, const std::string & label)
  {
    if (!status) {
      throw std::runtime_error(label + " failed for motor " + std::to_string(motor_id));
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<CalibrateNode>();
    node->run();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("calibrate"), "%s", error.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
