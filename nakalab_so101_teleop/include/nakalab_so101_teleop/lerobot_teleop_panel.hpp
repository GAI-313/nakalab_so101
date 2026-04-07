#ifndef NAKALAB_SO101_TELEOP__LEROBOT_TELEOP_PANEL_HPP_
#define NAKALAB_SO101_TELEOP__LEROBOT_TELEOP_PANEL_HPP_

#include <string>
#include <map>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rosbag2_cpp/writer.hpp>

class QPushButton;
class QLabel;
class QTabWidget;
class QTextEdit;

namespace nakalab_so101_teleop
{

class LeRobotTeleopPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit LeRobotTeleopPanel(QWidget * parent = nullptr);
  virtual ~LeRobotTeleopPanel() = default;

  void onInitialize() override;

protected Q_SLOTS:
  void onRecordingClicked();
  void onSaveClicked();
  void onResetClicked();
  void onCalibrateClicked();
  void onTeleopClicked();
  void onLoadConfigClicked();

protected:
  // Tab Widget
  QTabWidget * tab_widget_;

  // UI Elements (Operating Tab)
  QPushButton * teleop_button_;
  QPushButton * recording_button_;
  QPushButton * save_button_;
  QPushButton * reset_button_;
  QPushButton * calibrate_button_;
  QLabel * status_label_;

  // UI Elements (Info Tab)
  QTextEdit * config_display_;
  QPushButton * load_config_button_;

  // State
  bool is_recording_;
  bool is_teleop_active_;
  std::string current_config_path_;
  std::string last_bag_path_;
  YAML::Node last_recorded_config_;

  // ROS Node & Client
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr teleop_client_;

  // Rosbag recording
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;

  void startRecording();
  void stopRecording();

  void updateTeleopButtonUI();
  void loadYamlConfig(const std::string & path);
};

}  // namespace nakalab_so101_teleop

#endif  // NAKALAB_SO101_TELEOP__LEROBOT_TELEOP_PANEL_HPP_
