# nakalab_so101

SO-101 Driver for ROS2


## Setup

- [macOS で Teleop をする方法](/docs/seup_macos.md)
- [Build on Linux](/docs/setup_linux.md)

## Tutorials

1. [SO-101 をキャリブレーションする方法](/doc/calibration.md)


<!--
## Execute on macOS
- Find Port<br>
    You can search for the SO-101 device path.
    ```bash
    pixi run ros2 run nakalab_so101_py find_port
    ```

- Leader Arm<br>
    ```bash
    pixi run ros2 run nakalab_so101_py leader_arm_driver_node --ros-args -p device:=<device path>
    ```

## Execute on Linux
- Follower Arm
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args -p device:=/dev/ttyACM0
    ```
    Allow teleop by the Leader arm.
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args -p device:=/dev/ttyACM0 -r /follower/joint_commands:=/leader/joint_states
    ```
    ```bash
    ros2 service call /teleop std_srvs/srv/SetBool "data: true"
    ```
-->