#include "nakalab_so101_teleop/lerobot_teleop_panel.hpp"
#include "nakalab_so101_teleop/lerobot_exporter.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QFileDialog>
#include <QMessageBox>

#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace nakalab_so101_teleop
{

LeRobotTeleopPanel::LeRobotTeleopPanel(QWidget * parent)
: rviz_common::Panel(parent),
  is_recording_(false),
  is_teleop_active_(false)
{
  auto * main_layout = new QVBoxLayout(this);
  tab_widget_ = new QTabWidget(this);
  main_layout->addWidget(tab_widget_);

  // --- Operating Tab ---
  auto * operating_tab = new QWidget();
  auto * op_layout = new QVBoxLayout(operating_tab);

  auto * status_group = new QGroupBox("LeRobot Collection Status", operating_tab);
  auto * status_layout = new QVBoxLayout();
  status_label_ = new QLabel("Ready", status_group);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #4CAF50;");
  status_layout->addWidget(status_label_);
  status_group->setLayout(status_layout);
  op_layout->addWidget(status_group);

  auto * control_group = new QGroupBox("Control", operating_tab);
  auto * control_layout = new QHBoxLayout();
  teleop_button_ = new QPushButton("teleop", control_group);
  teleop_button_->setFixedHeight(40);
  teleop_button_->setStyleSheet("background-color: #bdbdbd; color: white; font-weight: bold;");
  connect(teleop_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onTeleopClicked);
  recording_button_ = new QPushButton("RECORDING", control_group);
  recording_button_->setFixedHeight(40);
  recording_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;");
  connect(recording_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onRecordingClicked);
  save_button_ = new QPushButton("SAVE", control_group);
  save_button_->setFixedHeight(40);
  save_button_->setEnabled(true);
  connect(save_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onSaveClicked);
  control_layout->addWidget(teleop_button_);
  control_layout->addWidget(recording_button_);
  control_layout->addWidget(save_button_);
  control_group->setLayout(control_layout);
  op_layout->addWidget(control_group);

  auto * extra_group = new QGroupBox("Utility", operating_tab);
  auto * extra_layout = new QHBoxLayout();
  reset_button_ = new QPushButton("RESET", extra_group);
  connect(reset_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onResetClicked);
  calibrate_button_ = new QPushButton("CALIBRATE", extra_group);
  connect(calibrate_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onCalibrateClicked);
  extra_layout->addWidget(reset_button_);
  extra_layout->addWidget(calibrate_button_);
  extra_group->setLayout(extra_layout);
  op_layout->addWidget(extra_group);
  op_layout->addStretch();
  tab_widget_->addTab(operating_tab, "Operating");

  // --- Info Tab ---
  auto * info_tab = new QWidget();
  auto * info_layout = new QVBoxLayout(info_tab);
  info_layout->addWidget(new QLabel("LeRobot Dataset / Topic Configuration:", info_tab));
  config_display_ = new QTextEdit(info_tab);
  config_display_->setReadOnly(true);
  config_display_->setLineWrapMode(QTextEdit::NoWrap);
  config_display_->setFontFamily("monospace");
  info_layout->addWidget(config_display_);
  load_config_button_ = new QPushButton("Load Config (YAML)", info_tab);
  connect(load_config_button_, &QPushButton::clicked, this, &LeRobotTeleopPanel::onLoadConfigClicked);
  info_layout->addWidget(load_config_button_);
  tab_widget_->addTab(info_tab, "Info");
}

void LeRobotTeleopPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  teleop_client_ = node_->create_client<std_srvs::srv::SetBool>("/teleop");
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("nakalab_so101_teleop");
    current_config_path_ = pkg_share + "/config/teleop_default_config.yaml";
    loadYamlConfig(current_config_path_);
  } catch (const std::exception & e) {
    config_display_->setPlainText(QString("Failed to initialize config: ") + e.what());
  }
}

void LeRobotTeleopPanel::onTeleopClicked()
{
  if (!teleop_client_->service_is_ready()) {
    status_label_->setText("ERR: /teleop not ready");
    status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #f44336;");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_teleop_active_;
  teleop_client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    if (future.get()->success) {
      is_teleop_active_ = !is_teleop_active_;
      updateTeleopButtonUI();
    }
  });
}

void LeRobotTeleopPanel::updateTeleopButtonUI() {
  teleop_button_->setStyleSheet(is_teleop_active_ ? 
    "background-color: #4CAF50; color: white; font-weight: bold;" : 
    "background-color: #bdbdbd; color: white; font-weight: bold;");
}

void LeRobotTeleopPanel::onRecordingClicked()
{
  if (!is_recording_) {
    startRecording();
  } else {
    stopRecording();
  }
}

void LeRobotTeleopPanel::startRecording()
{
  try {
    YAML::Node config = YAML::LoadFile(current_config_path_);
    std::string save_path = config["/**"]["ros__parameters"]["dataset"]["save_path"].as<std::string>();
    std::string repo_id = config["/**"]["ros__parameters"]["dataset"]["repo_id"].as<std::string>();

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
    std::string timestamp = ss.str();

    std::filesystem::path bag_root = std::filesystem::path(save_path) / repo_id / "rosbag";
    std::filesystem::path bag_dir = bag_root / (repo_id + "_" + timestamp);
    std::filesystem::create_directories(bag_dir.parent_path());

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_dir.string();
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options, converter_options);

    auto topic_names_and_types = node_->get_topic_names_and_types();

    auto start_topic_recording = [&](const std::string & topic_name) {
      if (topic_name.empty()) return;
      if (topic_names_and_types.find(topic_name) == topic_names_and_types.end()) {
        RCLCPP_WARN(node_->get_logger(), "Topic %s not found in graph. Skipping.", topic_name.c_str());
        return;
      }

      std::string type_name = topic_names_and_types[topic_name][0];

      rosbag2_storage::TopicMetadata meta;
      meta.name = topic_name;
      meta.type = type_name;
      meta.serialization_format = "cdr";
      writer_->create_topic(meta);

      auto sub = node_->create_generic_subscription(
        topic_name,
        type_name,
        rclcpp::SensorDataQoS(),
        [this, topic_name, type_name](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
          auto bag_msg = std::make_shared<rclcpp::SerializedMessage>(*msg);
          writer_->write(bag_msg, topic_name, type_name, node_->now());
        }
      );
      subscriptions_.push_back(sub);
    };

    auto params = config["/**"]["ros__parameters"];
    start_topic_recording(params["observation"]["joint_topic"].as<std::string>());
    start_topic_recording(params["observation"]["arm_camera_topic"].as<std::string>());
    start_topic_recording(params["action"]["joint_topic"].as<std::string>());

    if (params["observation"]["other_topics"].IsMap()) {
      for (auto it = params["observation"]["other_topics"].begin(); it != params["observation"]["other_topics"].end(); ++it) {
        start_topic_recording(it->second.as<std::string>());
      }
    }

    is_recording_ = true;
    last_bag_path_ = bag_dir.string();
    last_recorded_config_ = config;
    recording_button_->setText("STOP");
    recording_button_->setStyleSheet("background-color: #f44336; color: white; font-weight: bold;");
    status_label_->setText(QString("REC: ") + QString::fromStdString(timestamp));
    status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #f44336;");
    save_button_->setEnabled(false);
  } catch (const std::exception & e) {
    QMessageBox::critical(this, "Recording Error", e.what());
  }
}

void LeRobotTeleopPanel::stopRecording()
{
  subscriptions_.clear();
  if (writer_) {
    writer_->close();
    writer_.reset();
  }
  is_recording_ = false;
  recording_button_->setText("RECORDING");
  recording_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;");
  status_label_->setText("Stopped. Bag saved.");
  status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #FF9800;");
  save_button_->setEnabled(true);
}

void LeRobotTeleopPanel::onLoadConfigClicked()
{
  QString fileName = QFileDialog::getOpenFileName(this, "Open Config", "", "YAML (*.yaml)");
  if (!fileName.isEmpty()) loadYamlConfig(fileName.toStdString());
}

void LeRobotTeleopPanel::loadYamlConfig(const std::string & path)
{
  try {
    std::ifstream ifs(path);
    std::stringstream ss;
    ss << ifs.rdbuf();
    config_display_->setPlainText(QString::fromStdString(ss.str()));
    current_config_path_ = path;
    status_label_->setText("Config Loaded");
  } catch (...) { status_label_->setText("Config Load ERR"); }
}

void LeRobotTeleopPanel::onSaveClicked() { 
  if (last_bag_path_.empty()) {
    status_label_->setText("No bag to save");
    return;
  }
  
  status_label_->setText("Saving Dataset...");
  status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #2196F3;");
  
  try {
    LeRobotExporter exporter(last_recorded_config_);
    std::string save_path = last_recorded_config_["/**"]["ros__parameters"]["dataset"]["save_path"].as<std::string>();
    
    if (exporter.exportBag(last_bag_path_, save_path)) {
      status_label_->setText("Dataset SAVED");
      status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #4CAF50;");
    } else {
      status_label_->setText("Save FAILED");
      status_label_->setStyleSheet("font-weight: bold; font-size: 14pt; color: #f44336;");
    }
  } catch (const std::exception & e) {
    QMessageBox::critical(this, "Save Error", e.what());
    status_label_->setText("Save ERR");
  }
}
void LeRobotTeleopPanel::onResetClicked() { RCLCPP_INFO(node_->get_logger(), "Reset"); }
void LeRobotTeleopPanel::onCalibrateClicked() { RCLCPP_INFO(node_->get_logger(), "Calibrate"); }

}  // namespace nakalab_so101_teleop

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nakalab_so101_teleop::LeRobotTeleopPanel, rviz_common::Panel)
