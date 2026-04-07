#ifndef NAKALAB_SO101_TELEOP__LEROBOT_EXPORTER_HPP_
#define NAKALAB_SO101_TELEOP__LEROBOT_EXPORTER_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

namespace nakalab_so101_teleop
{

struct ExporterConfig {
    std::string repo_id;
    std::string task_name;
    std::string joint_topic;
    std::string action_topic;
    std::string arm_camera_topic;
    std::map<std::string, std::string> other_camera_topics;
    double fps = 30.0;
};

class LeRobotExporter
{
public:
    explicit LeRobotExporter(const YAML::Node & config_node);
    virtual ~LeRobotExporter() = default;

    bool exportBag(const std::string & bag_path, const std::string & target_dir);

private:
    ExporterConfig config_;

    int getNextEpisodeIndex(const std::filesystem::path & dataset_dir);
    void writeInfoJson(const std::filesystem::path & dataset_dir, int num_episodes, int total_frames);
    void appendEpisodeJsonl(const std::filesystem::path & dataset_dir, int episode_idx, int num_frames);
};

}  // namespace nakalab_so101_teleop

#endif  // NAKALAB_SO101_TELEOP__LEROBOT_EXPORTER_HPP_
