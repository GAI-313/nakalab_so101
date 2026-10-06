#include "nakalab_so101/HomingCalibration.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
const std::map<std::string, uint8_t> kMotors = {
  {"shoulder_pan", 1}, {"shoulder_lift", 2}, {"elbow_flex", 3},
  {"wrist_flex", 4}, {"wrist_roll", 5}, {"gripper", 6}
};

// 実位置を固定し，オフセット変更が現在位置へ反映されるサーボを模擬する．
struct FakeBus
{
  std::map<uint8_t, int> actual = {
    {1, 3000}, {2, 1300}, {3, 2700}, {4, 1000}, {5, 2500}, {6, 1900}
  };
  std::map<uint8_t, int> offsets = {
    {1, 1000}, {2, -500}, {3, 750}, {4, -1200}, {5, 500}, {6, -250}
  };
  std::map<std::pair<uint8_t, uint8_t>, uint16_t> registers;
  bool ignore_reset = false;
  bool ignore_new_offset = false;
  bool ignore_limits = false;
  bool stale_feedback = false;
  bool missing_position = false;

  bool write_homing_offset(uint8_t id, int value)
  {
    if (!((ignore_reset && value == 0) || (ignore_new_offset && value != 0))) {
      offsets[id] = value;
    }
    return true;
  }

  std::optional<int> read_homing_offset(uint8_t id)
  {
    return offsets.at(id);
  }

  bool write_u16(uint8_t id, uint8_t address, uint16_t value)
  {
    if (!ignore_limits) {
      registers[{id, address}] = value;
    }
    return true;
  }

  std::optional<uint16_t> read_u16(uint8_t id, uint8_t address)
  {
    const auto found = registers.find({id, address});
    if (found == registers.end()) {
      return std::nullopt;
    }
    return found->second;
  }

  std::map<uint8_t, int16_t> sync_read_present_positions(const std::vector<uint8_t> & ids)
  {
    std::map<uint8_t, int16_t> positions;
    for (const auto id : ids) {
      if (missing_position && id == 1) {
        continue;
      }
      const auto offset = stale_feedback ? 0 : offsets.at(id);
      positions[id] = (actual.at(id) - offset + 4096) % 4096;
    }
    return positions;
  }
};
}  // namespace

TEST(HomingCalibration, RepeatedCalibrationKeepsPhysicalPoseCentered)
{
  for (const bool initially_calibrated : {false, true}) {
    FakeBus bus;
    if (!initially_calibrated) {
      for (auto & [id, offset] : bus.offsets) {
        (void)id;
        offset = 0;
      }
    }
    for (int attempt = 0; attempt < 2; ++attempt) {
      const auto offsets = nakalab_so101::calibrate_homing_offsets(bus, kMotors);
      const auto positions = bus.sync_read_present_positions({1, 2, 3, 4, 5, 6});
      for (const auto & [name, id] : kMotors) {
        EXPECT_EQ(offsets.at(name), bus.actual.at(id) - 2047);
        EXPECT_EQ(positions.at(id), 2047);
        EXPECT_EQ(bus.read_u16(id, 9), 0);
        EXPECT_EQ(bus.read_u16(id, 11), 4095);
      }
    }
  }
}

TEST(HomingCalibration, RejectsFailedReset)
{
  FakeBus bus;
  bus.ignore_reset = true;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}

TEST(HomingCalibration, RejectsFailedRangeReset)
{
  FakeBus bus;
  bus.ignore_limits = true;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}

TEST(HomingCalibration, RejectsFailedOffsetWrite)
{
  FakeBus bus;
  bus.ignore_new_offset = true;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}

TEST(HomingCalibration, RejectsStalePositionFeedback)
{
  FakeBus bus;
  bus.stale_feedback = true;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}

TEST(HomingCalibration, RejectsMissingPosition)
{
  FakeBus bus;
  bus.missing_position = true;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}

TEST(HomingCalibration, RejectsUnrepresentableOffset)
{
  FakeBus bus;
  bus.actual[1] = 4095;
  EXPECT_THROW(nakalab_so101::calibrate_homing_offsets(bus, kMotors), std::runtime_error);
}
