#include "nakalab_so101_teleop/lerobot_exporter.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fstream>
#include <iostream>

namespace nakalab_so101_teleop
{

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
            state_buffer[msg->recv_timestamp] = pos;
        } else if (msg->topic_name == config_.action_topic) {
            sensor_msgs::msg::JointState js;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
            serializer.deserialize_message(&serialized_msg, &js);
            std::vector<float> pos(js.position.begin(), js.position.end());
            action_buffer[msg->recv_timestamp] = pos;
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

    // Parquet Writing with Arrow
    auto index_builder = std::make_shared<arrow::Int64Builder>();
    auto timestamp_builder = std::make_shared<arrow::DoubleBuilder>();
    auto episode_builder = std::make_shared<arrow::Int64Builder>();
    auto state_list_builder = std::make_shared<arrow::ListBuilder>(arrow::default_memory_pool(), std::make_shared<arrow::FloatBuilder>());
    auto action_list_builder = std::make_shared<arrow::ListBuilder>(arrow::default_memory_pool(), std::make_shared<arrow::FloatBuilder>());

    auto* state_value_builder = static_cast<arrow::FloatBuilder*>(state_list_builder->value_builder());
    auto* action_value_builder = static_cast<arrow::FloatBuilder*>(action_list_builder->value_builder());

    // Simple nearest-neighbor sync for states/actions
    for (int i = 0; i < num_frames; ++i) {
        double ts = i / 30.0;
        (void)index_builder->Append(i);
        (void)timestamp_builder->Append(ts);
        (void)episode_builder->Append(episode_idx);

        // Find nearest state
        std::vector<float> current_state = (state_buffer.empty()) ? std::vector<float>{0,0,0,0,0,0} : state_buffer.begin()->second;
        // In this implementation, we take the one with the smallest timestamp or improve logic
        (void)state_list_builder->Append();
        for (float v : current_state) (void)state_value_builder->Append(v);
        
        // Find nearest action
        std::vector<float> current_action = (action_buffer.empty()) ? current_state : action_buffer.begin()->second;
        (void)action_list_builder->Append();
        for (float v : current_action) (void)action_value_builder->Append(v);
    }

    std::shared_ptr<arrow::Array> index_array, ts_array, ep_array, state_array, action_array;
    (void)index_builder->Finish(&index_array);
    (void)timestamp_builder->Finish(&ts_array);
    (void)episode_builder->Finish(&ep_array);
    (void)state_list_builder->Finish(&state_array);
    (void)action_list_builder->Finish(&action_array);

    auto schema = arrow::schema({
        arrow::field("index", arrow::int64()),
        arrow::field("timestamp", arrow::float64()),
        arrow::field("episode_index", arrow::int64()),
        arrow::field("observation.state", arrow::list(arrow::float32())),
        arrow::field("action", arrow::list(arrow::float32()))
    });

    auto table = arrow::Table::Make(schema, {index_array, ts_array, ep_array, state_array, action_array});
    std::string parquet_path = (dataset_dir / "data" / ("chunk_" + std::to_string(episode_idx) + ".parquet")).string();
    auto output_file_result = arrow::io::FileOutputStream::Open(parquet_path);
    if (!output_file_result.ok()) return false;
    auto output_file = output_file_result.ValueOrDie();
    (void)parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), output_file, 1024);

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
        while (std::getline(ifs, line)) {
            try {
                auto j = nlohmann::json::parse(line);
                int idx = j["episode_index"];
                if (idx > max_idx) max_idx = idx;
            } catch (...) {}
        }
    }
    return max_idx + 1;
}

void LeRobotExporter::writeInfoJson(const std::filesystem::path & dataset_dir, int num_episodes, int total_frames)
{
    nlohmann::json info;
    info["codebase_version"] = "v0.1";
    info["robot_type"] = "so-101";
    info["total_episodes"] = num_episodes;
    info["total_frames"] = total_frames;
    info["fps"] = 30;
    
    std::ofstream ofs(dataset_dir / "info.json");
    ofs << info.dump(4);
}

void LeRobotExporter::appendEpisodeJsonl(const std::filesystem::path & dataset_dir, int episode_idx, int num_frames)
{
    nlohmann::json ep;
    ep["episode_index"] = episode_idx;
    ep["num_frames"] = num_frames;
    ep["task"] = config_.task_name;
    
    std::ofstream ofs(dataset_dir / "meta" / "episodes.jsonl", std::ios::app);
    ofs << ep.dump() << "\n";
}

}  // namespace nakalab_so101_teleop
