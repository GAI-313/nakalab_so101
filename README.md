# nakalab_so101

SO-101 Driver for ROS2

## Build
```bash
rosdep install -y -i --from-path src
```
```bash
colcon build --symlink-install --packages-select nakalab_so101
```

## Execute
- Follower Arm
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args -p device:=/dev/ttyACM0
    ```
