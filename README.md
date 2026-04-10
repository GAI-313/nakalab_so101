# nakalab_so101

SO-101 Driver for ROS2


## macOS で Teleop をする方法
1. 任意の場所に `nakalab_so101` をクローンします．
    ```bash
    git clone -b jazzy https://github.com/GAI-313/nakalab_so101.git
    ```

1. `nakalab_so101` に移動します．
    ```bash
    cd nakalab_so1o1
    ```
    
1. pixi がない場合は次のコマンドでインストールします．
    ```bash
    curl -fsSL https://pixi.sh/install.sh | sh
    ```
    インストールが完了したらターミナルを再起動してください．また次のステップに進む前に再起動したターミナルで `nakalab_so101` に移動してください．
    
1. [cyclonedds.xml](cyclonedds.xml) を編集して，ROS2 とコミュニケーションするためのネットワークインターフェースを定義します．<br>
    次のコマンドを実行し，利用したいネットワークインターフェース名を特定してください．
    ```bash
    ifconfig
    ```
    次のタグの `name` 属性にネットワークインターフェース名を記述してください．
    ```xml
    ...
        <NetworkInterface autodetermine="false" name="en11"/>
    ...
    ```

1. 次のコマンドで `nakalab_so101` をビルドします．
    ```bash
    pixi run colcon_build
    ```

1. インストールが完了したら，次のコマンドを実行して RViz にロボットが表示されることを確認してください．
    ```bash
    pixi run description
    ```
    正常にツールが起動することを確認したら，`nakalab_so101` が正常にインストールされたことを示しています．

1. SO-101 Leader アームを Mac に接続してください．
1. SO-101 Leader アームが Mac に接続されている状態で，次のコマンドを実行してください．
    ```bash
    pixi run find_port
    ```
    `Remove the USB cable from your MotorsBus and press Enter when done.` と表示されたら，SO-101 Leader アームを Mac から抜いて，エンターキーを押してください．するとアームのデバイスパスが検出されます．

1. SO-101 Leader アームを Mac に際接続し，

## Build on Linux
```bash
rosdep install -y -i --from-path src
```
```bash
colcon build --symlink-install --packages-select nakalab_so101
```

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
