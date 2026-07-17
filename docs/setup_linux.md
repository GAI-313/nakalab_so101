## Build on Linux
１．ROS2 をインストールし，以下のコマンドを実行してください．
    ```bash
    source /opt/ros/humble/setup.bash
    ```
１．ROS2 ワークスペースを作成します．
    ```bash
    cd && mkdir -p ~/colcon_ws/src
    ```
1. ROS2 ワークスペース内の `src` ディレクトリに移動します．
    ```bash
    cd ~/colcon_ws/src
    ```
１．このリポジトリを `src` ディレクトリ内にクローンします．
    ```bash
    git clone -b humble https://github.com/GAI-313/nakalab_so101.git
    ```
1. `nakalab_so101` が依存するパッケージを `src` ディレクトリ内にクローンします．
    ```bash
    vcs import . < nakalab_so101/depends.repos
    ```
1. その他必要な依存関係をインストールします．
    ```bash
    rosdep install -y -i --from-path .
    ```
1. ROS2 ワークスペース直下に移動します．
    ```bash
    cd ~/colcon_ws
    ```
1. 次のコマンドを実行して `nakalab_so101` をビルドします．
    ```bash
    colcon build --symlink-install --packages-up-to nakalab_so101
    ```
