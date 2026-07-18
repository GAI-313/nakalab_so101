#ifndef NAKALAB_SO101__CALIBRATION_HPP_
#define NAKALAB_SO101__CALIBRATION_HPP_

#include <map>
#include <string>
#include <vector>

namespace nakalab_so101
{

class Calibration
{
public:
  static constexpr const char * kFollowerDirectory = "so101_follower";

  static Calibration load_from_file(const std::string & path);
  static std::vector<std::string> find_follower_files(const std::string & calibration_root);

  int homing_offset(const std::string & joint_name) const;

private:
  std::map<std::string, int> homing_offsets_;
};

}  // namespace nakalab_so101

#endif  // NAKALAB_SO101__CALIBRATION_HPP_
