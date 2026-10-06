#ifndef NAKALAB_SO101__HOMING_CALIBRATION_HPP_
#define NAKALAB_SO101__HOMING_CALIBRATION_HPP_

#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace nakalab_so101
{

constexpr int kHomingCenterPosition = 2047;
constexpr int kHomingPositionTolerance = 10;

// トルク無効・EEPROM ロック解除後，基準姿勢を保持した状態で呼び出す．
// Bus を差し替えることで，実機なしで旧オフセットの影響を検証できる．
template<typename Bus>
std::map<std::string, int> calibrate_homing_offsets(
  Bus & bus, const std::map<std::string, uint8_t> & motors)
{
  std::vector<uint8_t> ids;
  for (const auto & [name, id] : motors) {
    ids.push_back(id);
    if (!bus.write_homing_offset(id, 0) ||
      !bus.write_u16(id, 9, 0) || !bus.write_u16(id, 11, 4095))
    {
      throw std::runtime_error(name + ": キャリブレーションのリセットに失敗しました．");
    }
    const auto offset = bus.read_homing_offset(id);
    const auto minimum = bus.read_u16(id, 9);
    const auto maximum = bus.read_u16(id, 11);
    if (!offset || *offset != 0 || !minimum || *minimum != 0 ||
      !maximum || *maximum != 4095)
    {
      throw std::runtime_error(name + ": キャリブレーションのリセットを確認できません．");
    }
  }

  // Present_Position = Actual_Position - Homing_Offset のため，リセット後に計測する．
  const auto positions = bus.sync_read_present_positions(ids);
  std::map<std::string, int> offsets;
  for (const auto & [name, id] : motors) {
    const auto position = positions.find(id);
    if (position == positions.end() || position->second < 0 || position->second > 4095) {
      throw std::runtime_error(name + ": 基準位置を正しく読み取れません．");
    }
    const int offset = position->second - kHomingCenterPosition;
    if (std::abs(offset) > 2047) {
      throw std::runtime_error(name + ": オフセットが表現可能な範囲を超えています．");
    }
    offsets[name] = offset;
  }

  for (const auto & [name, id] : motors) {
    if (!bus.write_homing_offset(id, offsets.at(name))) {
      throw std::runtime_error(name + ": オフセットの書き込みに失敗しました．");
    }
    const auto offset = bus.read_homing_offset(id);
    if (!offset || *offset != offsets.at(name)) {
      throw std::runtime_error(name + ": オフセットの書き込みを確認できません．");
    }
  }

  const auto centered_positions = bus.sync_read_present_positions(ids);
  for (const auto & [name, id] : motors) {
    const auto position = centered_positions.find(id);
    if (position == centered_positions.end() ||
      std::abs(position->second - kHomingCenterPosition) > kHomingPositionTolerance)
    {
      throw std::runtime_error(
              name + ": 補正後の基準位置が中央から外れています．"
              "基準姿勢を保持してキャリブレーションをやり直してください．");
    }
  }
  return offsets;
}

}  // namespace nakalab_so101

#endif  // NAKALAB_SO101__HOMING_CALIBRATION_HPP_
