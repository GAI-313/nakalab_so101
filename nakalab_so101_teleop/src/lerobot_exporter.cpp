#include "nakalab_so101_teleop/lerobot_exporter.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>
#include <type_traits>
#include <utility>

namespace nakalab_so101_teleop
{

namespace
{

std::string shellQuote(const std::string & value)
{
    std::string quoted = "'";
    for (const char c : value) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

std::string jsonEscape(const std::string & value)
{
    std::string escaped;
    for (const char c : value) {
        switch (c) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += c;
                break;
        }
    }
    return escaped;
}

void writeFloatArray(std::ofstream & ofs, const std::vector<float> & values)
{
    ofs << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) ofs << ",";
        ofs << values[i];
    }
    ofs << "]";
}

template<typename T, typename = void>
struct has_recv_timestamp : std::false_type {};

template<typename T>
struct has_recv_timestamp<T, std::void_t<decltype(std::declval<T>().recv_timestamp)>> : std::true_type {};

template<typename T>
int64_t bagMessageTimestamp(const T & msg)
{
    if constexpr (has_recv_timestamp<T>::value) {
        return msg.recv_timestamp;
    } else {
        return msg.time_stamp;
    }
}

}  // namespace

LeRobotExporter::LeRobotExporter(const YAML::Node & config_node)
{
    auto params = config_node["/**"]["ros__parameters"];
    config_.repo_id = params["dataset"]["repo_id"].as<std::string>();
    config_.task_name = params["dataset"]["task_name"].as<std::string>();
    config_.joint_topic = params["observation"]["joint_topic"].as<std::string>();
    config_.arm_camera_topic = params["observation"]["arm_camera_topic"].as<std::string>();
    
    // Add action topic from config
    if (params["action"]["joint_topic"]) {
        config_.action_topic = params["action"]["joint_topic"].as<std::string>();
    }

    if (params["observation"]["other_topics"].IsMap()) {
        for (auto it = params["observation"]["other_topics"].begin(); it != params["observation"]["other_topics"].end(); ++it) {
            config_.other_camera_topics[it->first.as<std::string>()] = it->second.as<std::string>();
        }
    }
}

bool LeRobotExporter::exportBag(const std::string & bag_path, const std::string & target_dir)
{
    std::filesystem::path dataset_dir = std::filesystem::path(target_dir) / config_.repo_id / "lerobot";
    std::filesystem::create_directories(dataset_dir / "data");
    std::filesystem::create_directories(dataset_dir / "videos");
    std::filesystem::create_directories(dataset_dir / "meta");

    int episode_idx = getNextEpisodeIndex(dataset_dir);

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "mcap";
    reader.open(storage_options);

    std::map<int64_t, std::vector<float>> state_buffer;
    std::map<int64_t, std::vector<float>> action_buffer;
    std::map<std::string, std::vector<cv::Mat>> image_streams;

    std::map<std::string, std::string> cameras = {{ "observation.images.laptop", config_.arm_camera_topic }};
    for(auto const& [name, topic] : config_.other_camera_topics) {
        cameras["observation.images." + name] = topic;
    }

    for(auto const& [name, topic] : cameras) {
        std::filesystem::create_directories(dataset_dir / "videos" / name);
    }

    while (reader.has_next()) {
        auto msg = reader.read_next();
        if (msg->topic_name == config_.joint_topic) {
            sensor_msgs::msg::JointState js;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
            serializer.deserialize_message(&serialized_msg, &js);
            std::vector<float> pos(js.position.begin(), js.position.end());
            state_buffer[bagMessageTimestamp(*msg)] = pos;
        } else if (msg->topic_name == config_.action_topic) {
            sensor_msgs::msg::JointState js;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
            serializer.deserialize_message(&serialized_msg, &js);
            std::vector<float> pos(js.position.begin(), js.position.end());
            action_buffer[bagMessageTimestamp(*msg)] = pos;
        }
        for(auto const& [cam_name, topic] : cameras) {
            if (msg->topic_name == topic) {
                sensor_msgs::msg::Image img;
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
                serializer.deserialize_message(&serialized_msg, &img);
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
                    image_streams[cam_name].push_back(cv_ptr->image.clone());
                } catch (...) {}
            }
        }
    }

    std::string main_cam = "observation.images.laptop";
    if (image_streams.find(main_cam) == image_streams.end() || image_streams[main_cam].empty()) return false;
    int num_frames = image_streams[main_cam].size();

    for(auto const& [cam_name, frames] : image_streams) {
        std::string video_path = (dataset_dir / "videos" / cam_name / ("episode_" + std::to_string(episode_idx) + ".mp4")).string();
        cv::VideoWriter writer(video_path, cv::VideoWriter::fourcc('m','p','4','v'), 30, frames[0].size());
        for(auto const& f : frames) writer.write(f);
        writer.release();
    }

    std::string parquet_input_path = (dataset_dir / "data" / ("chunk_" + std::to_string(episode_idx) + ".json")).string();
    std::string parquet_path = (dataset_dir / "data" / ("chunk_" + std::to_string(episode_idx) + ".parquet")).string();

    {
        std::ofstream ofs(parquet_input_path);
        if (!ofs) return false;
        ofs << "[";
        for (int i = 0; i < num_frames; ++i) {
            if (i > 0) ofs << ",";
            double ts = i / 30.0;
            std::vector<float> current_state = (state_buffer.empty()) ? std::vector<float>{0,0,0,0,0,0} : state_buffer.begin()->second;
            std::vector<float> current_action = (action_buffer.empty()) ? current_state : action_buffer.begin()->second;

            ofs << "{\"index\":" << i
                << ",\"timestamp\":" << ts
                << ",\"episode_index\":" << episode_idx
                << ",\"observation.state\":";
            writeFloatArray(ofs, current_state);
            ofs << ",\"action\":";
            writeFloatArray(ofs, current_action);
            ofs << "}";
        }
        ofs << "]";
    }

    std::filesystem::path writer_path =
        std::filesystem::path(ament_index_cpp::get_package_prefix("nakalab_so101_teleop")) /
        "lib" / "nakalab_so101_teleop" / "write_lerobot_parquet";
    std::string command =
        shellQuote(writer_path.string()) + " " +
        shellQuote(parquet_input_path) + " " +
        shellQuote(parquet_path);

    if (std::system(command.c_str()) != 0) {
        return false;
    }

    std::error_code remove_error;
    std::filesystem::remove(parquet_input_path, remove_error);

    appendEpisodeJsonl(dataset_dir, episode_idx, num_frames);
    writeInfoJson(dataset_dir, episode_idx + 1, num_frames);

    return true;
}

int LeRobotExporter::getNextEpisodeIndex(const std::filesystem::path & dataset_dir)
{
    int max_idx = -1;
    if (std::filesystem::exists(dataset_dir / "meta" / "episodes.jsonl")) {
        std::ifstream ifs(dataset_dir / "meta" / "episodes.jsonl");
        std::string line;
        std::regex episode_index_regex("\"episode_index\"\\s*:\\s*(\\d+)");
        while (std::getline(ifs, line)) {
            std::smatch match;
            if (std::regex_search(line, match, episode_index_regex)) {
                int idx = std::stoi(match[1].str());
                if (idx > max_idx) max_idx = idx;
            }
        }
    }
    return max_idx + 1;
}

void LeRobotExporter::writeInfoJson(const std::filesystem::path & dataset_dir, int num_episodes, int total_frames)
{
    std::ofstream ofs(dataset_dir / "info.json");
    ofs << "{\n"
        << "    \"codebase_version\": \"v0.1\",\n"
        << "    \"robot_type\": \"so-101\",\n"
        << "    \"total_episodes\": " << num_episodes << ",\n"
        << "    \"total_frames\": " << total_frames << ",\n"
        << "    \"fps\": 30\n"
        << "}";
}

void LeRobotExporter::appendEpisodeJsonl(const std::filesystem::path & dataset_dir, int episode_idx, int num_frames)
{
    std::ofstream ofs(dataset_dir / "meta" / "episodes.jsonl", std::ios::app);
    ofs << "{\"episode_index\":" << episode_idx
        << ",\"num_frames\":" << num_frames
        << ",\"task\":\"" << jsonEscape(config_.task_name) << "\"}\n";
}

}  // namespace nakalab_so101_teleop
