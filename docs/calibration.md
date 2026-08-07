# SO-101 をキャリブレーションする方法

> [!IMPORTANT]
> [lerobot](https://github.com/huggingface/lerobot) にてすでに SO-101 アームのキャリブレーションを実施している場合でも，本リポジトリで再度キャリブレーションを実施してください．


1. 以下のコマンドを実行して，`calibrate` ノードが実行可能であることを確認してください．
    ```bash
    ros2 pkg executables nakalab_so101_py
    ```
    もし，`calibrate` が存在しない場合はパッケージのビルドに失敗している可能性があります．セットアップからやり直してください．
1. アームと PC を接続してください．
1. 以下のコマンドを実行してアームのキャリブレーションを実施します．
    ```bash
    ros2 run nakalab_so101_py calibrate --ros-args \
    -p device:=<Device Path> \
    -p arm_type:=<so101_leader | so101_follower> \
    -p calibration_id:=<任意の名前>
    ```
    例えば，アームが `/dev/ttyACM0` として認識されており，フォロワーアームを `nakalab_so101_follower` という名前でキャリブレーションを行いたい場合は以下のコマンドを実行します．
     ```bash
    ros2 run nakalab_so101_py calibrate --ros-args \
    -p device:=/dev/ttyACM0 \
    -p arm_type:=<so101_follower \
    -p calibration_id:=nakalab_so101_follower
    ```
1. アームを下記の画像のような姿勢に変更してください．<br>
    ```
    Move the arm to the middle of its range of motion, then press Enter.
    ```
    このようなログが表示されたら下記の画像のようにアームのすべてのジョイントが０度の姿勢にしてください．<br>
    <img src="https://i.imgur.com/GfS4Drl.png"/>
    <br>
    初期姿勢にできたらエンターキーを押してください．

1. 各ジョイントを回転させてください．このときすべてのジョイントが回転限界まで回し切ることが重要です．回しきったらエンターキーを押してください．するとキャリブレーションファイルが生成されて保存されます．

1. キャリブレーション完了後，ROS2 ワークスペース直下で以下のコマンドを実施して `nakalab_so101_description` をビルドしてください．
    ```bash
    colcon build --symlink-install nakalab_so101_description
    ```

1. Leader arm, Follower Arm それぞれこの作業を実施してください．

# Follower Arm のキャリブレーション結果を確認する方法

Follower Arm キャリブレーション後以下の作業を実施して動作確認及びキャリブレーション結果を確認できます．

1. 以下のコマンドを実施して `/joint_state` トピックからアーム制御を受け付けるようアームを起動します．
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args \
    -p device:=/dev/ttyACM0 \
    -r /follower/joint_commands:=/joint_states
    ```
    このとき，以下のようなメッセージが表示された場合キャリブレーションが正常に反映されていることを示しています．
    ```
    [WARN] [1784399610.848976296] [follower_arm_driver_node]: calibration_file is empty; automatically using /.../nakalab_so101_description/share/nakalab_so101_description/calibration/so101_follower/XXXX.json
    ```
1. 別のターミナルで次のコマンドを実行して RViz と joint_state_publisher_gui 経由でアームの操作，姿勢確認を行います． <br>
    **このコマンドを実行すると現在の姿勢からジョイント角度が全て０度の姿勢に素早く移動します．注意してください．**
    ```bash
    ros2 launch nakalab_so101_description display.launch.py use_rviz:=true use_jspg:=true
    ```
1. 起動後，アームと RViz 画面上のアームの姿勢が一致していることを確認してください．