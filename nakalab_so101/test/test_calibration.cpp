#include "nakalab_so101/Calibration.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace
{
const char * kJoints[] = {
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
};

std::string valid_json(int offset = 10)
{
  std::string result = "{";
  for (size_t i = 0; i < 6; ++i) {
    if (i != 0) {
      result += ",";
    }
    result += "\"" + std::string(kJoints[i]) + "\":{\"homing_offset\":" +
      std::to_string(offset + static_cast<int>(i)) + "}";
  }
  return result + "}";
}

class CalibrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() /
      ("nakalab_so101_calibration_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(root_ / "so101_follower");
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  std::filesystem::path root_;
};
}

TEST_F(CalibrationTest, LoadsArbitraryFileAndOffsets)
{
  const auto path = root_ / "so101_follower" / "my_arm.json";
  std::ofstream(path) << valid_json();
  const auto calibration = nakalab_so101::Calibration::load_from_file(path.string());
  EXPECT_EQ(calibration.homing_offset("shoulder_pan"), 10);
  EXPECT_EQ(calibration.homing_offset("gripper"), 15);
}

TEST_F(CalibrationTest, RejectsMalformedOrIncompleteFiles)
{
  EXPECT_THROW(
    nakalab_so101::Calibration::load_from_file(
      (root_ / "so101_follower" / "does_not_exist.json").string()),
    std::runtime_error);

  const auto malformed = root_ / "so101_follower" / "malformed.json";
  std::ofstream(malformed) << "not json";
  EXPECT_THROW(nakalab_so101::Calibration::load_from_file(malformed.string()), std::runtime_error);

  const auto incomplete = root_ / "so101_follower" / "incomplete.json";
  std::ofstream(incomplete) << "{\"shoulder_pan\":{\"homing_offset\":1}}";
  EXPECT_THROW(nakalab_so101::Calibration::load_from_file(incomplete.string()), std::runtime_error);
}

TEST_F(CalibrationTest, FindsOnlyFollowerCandidates)
{
  std::filesystem::create_directories(root_ / "so101_leader");
  std::ofstream(root_ / "so101_leader" / "leader.json") << valid_json();
  EXPECT_TRUE(nakalab_so101::Calibration::find_follower_files(root_.string()).empty());

  std::ofstream(root_ / "so101_follower" / "first.json") << valid_json();
  auto files = nakalab_so101::Calibration::find_follower_files(root_.string());
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files.front(), (root_ / "so101_follower" / "first.json").string());

  std::ofstream(root_ / "so101_follower" / "second.json") << valid_json();
  EXPECT_EQ(nakalab_so101::Calibration::find_follower_files(root_.string()).size(), 2u);
}
