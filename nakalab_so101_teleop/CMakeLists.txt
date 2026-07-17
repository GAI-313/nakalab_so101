cmake_minimum_required(VERSION 3.8)
project(nakalab_so101_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

add_library(lerobot_teleop_panel SHARED
  src/lerobot_teleop_panel.cpp
  src/lerobot_exporter.cpp
  include/nakalab_so101_teleop/lerobot_teleop_panel.hpp
)
target_include_directories(lerobot_teleop_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lerobot_teleop_panel
  rclcpp
  std_srvs
  rviz_common
  rviz_rendering
  ament_index_cpp
  rosbag2_cpp
  cv_bridge
  sensor_msgs
)

target_link_libraries(lerobot_teleop_panel
  Qt5::Widgets
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/write_lerobot_parquet
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lerobot_teleop_panel
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
