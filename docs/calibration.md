# SO-101 をキャリブレーションする方法

> [!IMPORTANT]
> [lerobot](https://github.com/huggingface/lerobot) にてすでに SO-101 アームのキャリブレーションを実施している場合でも，本リポジトリで再度キャリブレーションを実施してください．

## キャリブレーションの準備 <a id="prepare"></a>

実機操作の前に，作業者がアームを支えられる状態であることを確認してください．同じシリアルポートを使用するドライバを停止してから実施します．制御モードのドライバ停止とキャリブレーション開始時にはトルクが解除されます．

Python 版の `nakalab_so101_py calibrate` と C++ 版の `nakalab_so101 calibrate` は，基準姿勢で Enter を押した後，旧オフセットを 0，可動範囲を 0〜4095 に戻して読み戻します．その後に基準位置を計測し，新オフセットを書き込んで，現在位置が 2047 ±10 ステップ以内（約 ±0.88 度）になったことを確認します．可動範囲を動かす案内が出るまで，基準姿勢を保持してください．検証に失敗した場合は JSON を保存せず終了しますが，サーボ内の設定は途中まで変更されている可能性があるため，再度キャリブレーションが必要です．

旧オフセットをリセットしない実装で作成した校正結果は，ソフトウェアの更新だけでは修復されません．修正版をビルドしてから，基準姿勢と可動範囲を再計測してください．既存の JSON にオフセットを手動で加減しないでください．

この手順は，[LeRobot のリセット・基準位置計測](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py) と，[Feetech の位置変換式](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py) に基づきます．メーカーの [STS3215 の説明](https://www.feetechrc.com/20210430-56680.html) にも，現在位置を中央へ補正する機能が説明されています．

## キャリブレーションを実施する <a id="calibrate"></a>

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
    -p arm_type:=so101_follower \
    -p calibration_id:=nakalab_so101_follower
    ```
1. アームを下記の画像のような姿勢に変更してください．<br>
    ```text
    Move the arm to the middle of its range of motion, then press Enter.
    ```
    このようなログが表示されたら下記の画像のようにアームのすべてのジョイントが０度の姿勢にしてください．<br>
    <img src="https://i.imgur.com/GfS4Drl.png"/>
    <br>
    初期姿勢にできたらエンターキーを押してください．

1. 各ジョイントを回転させてください．このときすべてのジョイントが回転限界まで回し切ることが重要です．回しきったらエンターキーを押してください．するとキャリブレーションファイルが生成されて保存されます．

1. キャリブレーション完了後，ROS2 ワークスペース直下で以下のコマンドを実施して `nakalab_so101_description` をビルドしてください．
    ```bash
    colcon build --symlink-install --packages-select nakalab_so101_description
    ```

1. Leader arm, Follower Arm それぞれこの作業を実施してください．

## 実機の姿勢を表示する <a id="observe"></a>

作業者が実機の状態を確認した後，次のコマンドで実測した関節状態を `/joint_states` に出力します．C++ の `follower_arm_driver_node` は既定で `read_only:=true` です．このモードでは起動・終了時とも目標位置やトルクを書き込まず，関節指令も受け付けません．すでに有効なトルクを解除する機能ではないため，実機の状態は別途確認してください．

```bash
ros2 run nakalab_so101 follower_arm_driver_node --ros-args \
  -p device:=/dev/ttyACM0 \
  -p read_only:=true \
  -r /follower/joint_states:=/joint_states
```

別のターミナルで表示を開始します．実測状態と競合する状態を配信しないよう，`use_jspg:=false` を指定してください．

```bash
ros2 launch nakalab_so101_description display.launch.py use_rviz:=true use_jspg:=false
```

実機を基準姿勢にしたときの表示と，各関節を動かしたときの表示の方向を確認します．`Verified calibration JSON against motor homing offsets` は設定値の一致を示すログであり，実姿勢とロボットモデルの一致を保証するものではありません．

起動ログには，各関節の `raw`（実測位置），`rad`（配信角度），`goal`（保存された目標），`limits`（サーボ内の可動範囲），`torque`，`mode`，`phase` を出力します．姿勢が合わない場合は，再度制御を有効にする前に，このログと実機の基準姿勢を照合してください．

## 起動拒否とサーボ異常を確認する <a id="diagnostics"></a>

- `現在位置が可動範囲外です` は，実測した `raw` がサーボ内の `limits` を超えている状態です．ログに超過ステップ数と角度を表示します．範囲外の現在位置を保持目標として送らないよう，制御開始を拒否します．起動時の姿勢変化に対する許容誤差とは別の判定です．校正時の可動範囲の計測と，起動時の姿勢を確認してください．
- `error=1 (入力電圧異常)` はサーボが報告した電圧異常です．USB 接続とは別に，サーボ用電源の接続・通電状態を確認してください．角度の許容誤差では解消しません．低電圧・過電圧のどちらかは，このビットだけでは判別できません．異常ビットの意味は [Feetech 公式 SDK](https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/protocol_packet_handler.py) に従います．

## GUI から実機を操作して確認する <a id="command"></a>

> [!WARNING]
> 以下は実機を動かす手順です．表示専用のドライバを停止し，作業者が可動域と起動時の移動を確認してから実施してください．

制御には明示的な `read_only:=false` が必要です．ドライバは全関節のトルクが無効であること，サーボの応答 ID・エラー・データ長，動作モード，可動範囲，静止状態，目標位置の読み戻しを検証してからトルクを有効にします．起動時にトルクが有効な関節がある場合は設定を変更せず終了します．これらの検証は実機とロボットモデルの姿勢一致を保証するものではありません．

Follower Arm キャリブレーション後以下の作業を実施して動作確認及びキャリブレーション結果を確認できます．

1. 以下のコマンドを実施して `/joint_states` トピックからアーム制御を受け付けるようアームを起動します．
    ```bash
    ros2 run nakalab_so101 follower_arm_driver_node --ros-args \
    -p device:=/dev/ttyACM0 \
    -p read_only:=false \
    -r /follower/joint_commands:=/joint_states
    ```
    このとき，以下のメッセージは JSON が自動選択されたことを示します．姿勢の一致は別途確認してください．
    ```text
    [WARN] [1784399610.848976296] [follower_arm_driver_node]: calibration_file is empty; automatically using /.../nakalab_so101_description/share/nakalab_so101_description/calibration/so101_follower/XXXX.json
    ```
1. 別のターミナルで次のコマンドを実行して RViz と joint_state_publisher_gui 経由でアームの操作，姿勢確認を行います． <br>
    **このコマンドを実行すると現在の姿勢からジョイント角度が全て０度の姿勢に素早く移動します．注意してください．**
    ```bash
    ros2 launch nakalab_so101_description display.launch.py use_rviz:=true use_jspg:=true
    ```
1. 起動後，アームと RViz 画面上のアームの姿勢が一致していることを確認してください．
