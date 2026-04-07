cmake_minimum_required(VERSION 3.8)
project(nakalab_so101)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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
ament_target_dependencies(follower_arm_driver_node rclcpp sensor_msgs)
target_link_libraries(follower_arm_driver_node nakalab_so101_feetech_bus)

#
# Install targets
#
install(
  TARGETS
    nakalab_so101_feetech_bus
    leader_arm_driver_node
    follower_arm_driver_node
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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
