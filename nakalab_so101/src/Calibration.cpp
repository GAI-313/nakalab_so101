#include "nakalab_so101/Calibration.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <stdexcept>

namespace
{
constexpr const char * kJointNames[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
};
}

namespace nakalab_so101
{

Calibration Calibration::load_from_file(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("Unable to open calibration file: " + path);
  }

  boost::property_tree::ptree document;
  try {
    boost::property_tree::read_json(input, document);
  } catch (const std::exception & error) {
    throw std::runtime_error(
            "Invalid calibration JSON in " + path + ": " + std::string(error.what()));
  }

  Calibration calibration;
  for (const auto * joint_name : kJointNames) {
    const auto joint = document.get_child_optional(joint_name);
    const auto offset = joint ? joint->get_optional<int>("homing_offset") :
      boost::optional<int>();
    if (!offset) {
      throw std::runtime_error(
              "Calibration JSON is missing homing_offset for joint '" + std::string(joint_name) +
              "': " + path);
    }
    calibration.homing_offsets_[joint_name] = *offset;
  }

  return calibration;
}

std::vector<std::string> Calibration::find_follower_files(const std::string & calibration_root)
{
  std::vector<std::string> files;
  if (calibration_root.empty()) {
    return files;
  }

  const std::filesystem::path directory =
    std::filesystem::path(calibration_root) / kFollowerDirectory;
  std::error_code error;
  if (!std::filesystem::is_directory(directory, error)) {
    return files;
  }

  for (const auto & entry : std::filesystem::directory_iterator(directory, error)) {
    if (error) {
      break;
    }
    if (entry.is_regular_file(error) && entry.path().extension() == ".json") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

int Calibration::homing_offset(const std::string & joint_name) const
{
  const auto offset = homing_offsets_.find(joint_name);
  return offset == homing_offsets_.end() ? 0 : offset->second;
}

}  // namespace nakalab_so101
