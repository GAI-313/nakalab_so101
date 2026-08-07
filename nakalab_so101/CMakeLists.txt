cmake_minimum_required(VERSION 3.8)
project(nakalab_so101)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL libserial REQUIRED)

#
# Library for FeetechBus
#
add_library(nakalab_so101_feetech_bus src/FeetechBus.cpp)
target_include_directories(nakalab_so101_feetech_bus
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  SYSTEM
    ${LIBSERIAL_INCLUDE_DIRS}
)
target_link_libraries(nakalab_so101_feetech_bus
  PUBLIC
    ${LIBSERIAL_LIBRARIES}
)

add_library(nakalab_so101_calibration src/Calibration.cpp)
target_include_directories(nakalab_so101_calibration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#
# Leader Arm Driver Node
#

add_executable(leader_arm_driver_node src/leader_arm_driver_node.cpp)
ament_target_dependencies(leader_arm_driver_node rclcpp sensor_msgs std_srvs)
target_link_libraries(leader_arm_driver_node nakalab_so101_feetech_bus)

#
# Follower Arm Driver Node
#
add_executable(follower_arm_driver_node src/follower_arm_driver_node.cpp)
ament_target_dependencies(follower_arm_driver_node ament_index_cpp rclcpp sensor_msgs)
target_link_libraries(follower_arm_driver_node nakalab_so101_feetech_bus nakalab_so101_calibration)

#
# Calibration Node
#
add_executable(calibrate src/calibrate_node.cpp)
ament_target_dependencies(calibrate ament_index_cpp rclcpp)
target_link_libraries(calibrate nakalab_so101_feetech_bus)

#
# Install targets
#
install(
  TARGETS
    nakalab_so101_feetech_bus
    nakalab_so101_calibration
    leader_arm_driver_node
    follower_arm_driver_node
    calibrate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  target_link_libraries(test_calibration nakalab_so101_calibration)
  target_include_directories(test_calibration PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
