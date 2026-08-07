# フォロワーアームの起動方法

## 基本

1. **SO-101 フォロワーアームと PC を接続する**<br>
    　以下のように必要なコンポーネントを接続します．
    ```mermaid
    flowchart TD
        A{Power} -->|12V Power| B(SO-101 Follower Arm)
        B <-->|USB Type-C| C(Your PC)  
    ```

1. **アームのデバイス権限を昇格させる**<br>
    　アームとのシリアル通信を可能にするために権限を昇格させます．
    ```bash
    sudo chmod 777 /dev/ttyACM0
    ```

1. **[事前確認] アームのキャリブレーションをする**<br>
    　フォロワーアームのキャリブレーションを行っていない場合，[SO-101 をキャリブレーションする方法](./calibration.md) を実施してください．
    　
1. **フォロワーアーム制御ノードを起動する**<br>
    　次のコマンドでフォロワーアームを起動します．パラメータ `device` にアームのデバイスパスを指定します．
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args \
    -p device:=/dev/ttyACM0
    ```
    起動が成功すると以下のログが出力され，アームのジョイントにトルクがかかります．
    ```
    [INFO] [xxx.xxx] [follower_arm_driver_node]: Follower Arm Driver started on /dev/ttyACM0
    ```

## 動作確認
　別のターミナルで次のコマンドらを実行すると，フォロワーアームのグリッパーが開閉します．
```bash
# グリッパーを 1.0 rad 開く
ros2 topic pub --once /follower/joint_commands sensor_msgs/msg/JointState "{name: [gripper], position: [1.0]}"
```
```bash
# グリッパーを 0.0 rad 開く
ros2 topic pub --once /follower/joint_commands sensor_msgs/msg/JointState "{name: [gripper], position: [0.0]}"
```

## 任意のトピック名にリマッピングする方法

　`follower_arm_driver_node` はデフォルトで [sensor_msgs/msg/JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) `/follower/joint_commands` をサブスクライブして各関節を制御します．以下のコマンドのようにこのトピックをリマッピングすることで任意のトピックからアームの関節を制御できます．
```bash
ros2 run nakalab_so101 follower_arm_driver_node --ros-args \
-p device:=/dev/ttyACM0 \
-r /follower/joint_commands:=<任意のトピック名>
```
