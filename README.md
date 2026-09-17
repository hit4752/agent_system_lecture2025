# lecture2025

ROS 1 と Choreonoid を用いたロボット制御・学習済み方策の推論の教材です。

| パッケージ | 内容 |
| --- | --- |
| `choreonoid_tutorial` | 姿勢制御、ROS 通信、LibTorch 推論の SimpleController、`RobotObservation` メッセージ、シミュレーションプロジェクト |
| `inference_tutorial` | TorchScript モデルを実行する C++ サンプル、学習結果のエクスポートスクリプト |

## ワークスペースの構築

対象環境は Ubuntu 20.04 / ROS Noetic、C++17 です。ROS Noetic をインストール済みの環境で実行してください。Choreonoid は通常の CMake パッケージを含むため、ここでは `catkin build` を使い、全パッケージを共通の `install` にインストールします。

### 1. ソースの取得

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install git python3-vcstool python3-catkin-tools python3-rosdep

mkdir -p ~/catkin_ws/agent_system_ws/src
cd ~/catkin_ws/agent_system_ws/src
git clone -b main https://github.com/agent-system/lecture2025.git lecture2025
cd ..
```

すでに `src/lecture2025` だけをクローンしてある場合は、上記の clone を省略し、ワークスペース直下から続けます。

```bash
vcs import src < src/lecture2025/.rosinstall
```

`vcstool` は `.rosinstall` 形式を読み込め、親リポジトリの取得後に `ext` 以下のリポジトリを取得します。`wstool` は入れ子の Git リポジトリを拒否するため、この構成では使用しません。`lecture2025/.rosinstall` 内のパスは `src` 基準です。既存の `lecture2025` 自体は上書き・再取得しません。

```text
src/
├── lecture2025/
├── simple_bipedal/                 # master
└── choreonoid/                     # release-2.2
    └── ext/
        ├── vnoid/                 # main（ショートトラック等）
        └── jaxon2_sample/         # main（JAXON2 サンプル）
```

バージョンはすべてブランチ名です。ブランチの更新に伴って取得内容も変わるため、過去の特定コミットを固定する構成ではありません。JAXON2 サンプルは取得しますが、上流の既定設定ではビルドは無効です。

### 2. 依存ライブラリ

`rosdep` を初めて使う環境だけ `sudo rosdep init` を実行し、続いて次を実行します。

```bash
rosdep update --include-eol-distros
# PyTorch は学習環境とバージョン・ABIを合わせて別途準備する
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y \
  --skip-keys="python3-pytorch-pip"
bash src/choreonoid/misc/script/install-requisites-ubuntu-20.04.sh
```

C++ の推論には **LibTorch** が必要です。既存の ROS / Choreonoid とリンクするため、Linux の **cxx11 ABI** 版を使用します。モデルを生成した PyTorch と互換性のあるバージョンを選んでください。CPU 版でこの教材の C++ 推論を実行できます。配布物と CMake の設定方法は [PyTorch の LibTorch 導入手順](https://docs.pytorch.org/cppdocs/installing.html) を参照してください。

LibTorch を例えば `~/genesis_ws/libtorch` に展開し、次のファイルがあることを確認します。CPU / cxx11 ABI の 2.7.1 を使用する例は次のとおりです（学習環境に合わせて版を変更してください）。

```bash
sudo apt install wget unzip
mkdir -p ~/genesis_ws
cd ~/genesis_ws
wget -O libtorch-2.7.1-cpu.zip 'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcpu.zip'
unzip libtorch-2.7.1-cpu.zip
cd ~/catkin_ws/agent_system_ws
```

```bash
test -f ~/genesis_ws/libtorch/share/cmake/Torch/TorchConfig.cmake
```

`package.xml` には Python スクリプトの PyTorch 依存を `python3-pytorch-pip` として記載しています。C++ LibTorch にはこの環境の rosdep に対応するキーがないため、`find_package(Torch REQUIRED)` で検出します。LibTorch 単体には Python の `torch` は含まれません。エクスポートスクリプトを使う場合は、学習に用いた Python 環境に `torch` と `PyYAML`、さらに Genesis とその学習環境に対応した `rsl_rl` を準備してください。

### 3. ビルド

ワークスペース直下で実行します。`Torch_DIR` は展開先に合わせて変更できます。既存の Python 版 PyTorch に付属する CMake 設定を使うこともできますが、C++ ABI と CUDA 等の依存が一致している必要があります。

```bash
source /opt/ros/noetic/setup.bash
catkin init
catkin config --install --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DTorch_DIR="$HOME/genesis_ws/libtorch/share/cmake/Torch"
catkin build -j2 -p1
source install/setup.bash
```

推論コードのコンパイルはメモリを多く使用します。メモリ不足の場合は `-j1 -p1` に下げてください。Choreonoid 本体と追加コントローラは同じ `install` 配下に配置されます。新しい端末でも `source /opt/ros/noetic/setup.bash` と、このワークスペースの `install/setup.bash` を読み込みます。

## 使い方

### 二脚モデルの表示

```bash
roslaunch simple_bipedal display.launch
```

RViz で Fixed Frame を `base_link` に設定し、RobotModel を追加します。Joint State Publisher GUI で関節角を変更できます。詳細は隣のリポジトリの [README](../simple_bipedal/README.md) を参照してください。

### Choreonoid

ワークスペース直下から、使用したいプロジェクトを開きます。

```bash
./install/bin/choreonoid src/lecture2025/choreonoid_tutorial/projects/simple_bipedal_inference.cnoid
# ショートトラック
./install/bin/choreonoid src/lecture2025/choreonoid_tutorial/projects/shorttrack_simple_bipedal.cnoid
```

モデルの読み込み後、シミュレーション開始ボタンで実行します。推論プロジェクトは次項の学習済みデータを先に準備してください。`.cnoid` は兄弟リポジトリを相対パスで参照するので、上記の `src` 構成を維持してソース内のプロジェクトを開いてください。

| プロジェクト | コントローラ / 用途 |
| --- | --- |
| `go2_stand.cnoid` | `StandController` による姿勢制御 |
| `go2_sample.cnoid` | `ROSChoreonoidBridgeController` による ROS 通信 |
| `go2_inference.cnoid` | `InferenceController` による Go2 推論 |
| `simple_bipedal_inference.cnoid` | `InferenceControllerSB` による二脚推論 |
| `shorttrack_simple_bipedal.cnoid` | 二脚推論と vnoid のショートトラック |

Go2 の URDF は別の Genesis 環境を参照します。`go2_sample` / `go2_inference` は `~/genesis_ws/Genesis/genesis/assets/urdf/go2/urdf/go2.urdf`、`go2_stand` は `~/genesis_ws/venv_genesis/src/genesis-world/genesis/assets/urdf/go2/urdf/go2.urdf` を参照します。配置が異なる場合はプロジェクト内の `file` を実際の URDF のパスに変更してください。

ROS 通信のサンプルでは別端末で `roscore` を起動してからシミュレーションを開始します。観測は `/server2client`（`choreonoid_tutorial/RobotObservation`）、入力は `/client2server` です。型は `rostopic info /client2server` で確認できます。

### 学習済みモデルの準備・推論

`.rosinstall` が再構築するのは ROS ワークスペースのソース構成です。Genesis、学習用リポジトリ、学習ログ・重みは含まれません。エクスポートスクリプトには次の外部環境への参照があります。

- Go2: `~/genesis_ws/Genesis/examples/locomotion/go2_env.py`
- 二脚: `~/genesis_ws/sandbox/genesis_simple_bipedal_rl/simple_bipedal_env.py`

学習に使用した Python 環境で、`cfgs.pkl` と `model_<番号>.pt` を含むログディレクトリを指定します。以下は二脚の例です。`100` は実在するチェックポイント番号に変更してください。

```bash
# ワークスペース直下、学習用 Python 環境を有効化済みとして実行
training_log="$HOME/genesis_ws/sandbox/genesis_simple_bipedal_rl/logs/simple_bipedal_walking"
python src/lecture2025/inference_tutorial/scripts/sb_dump_training_data.py \
  --log_dir "$training_log" --ckpt 100
python src/lecture2025/inference_tutorial/scripts/dump_cfgs_to_yaml.py \
  --log_dir "$training_log"
mkdir -p "$training_log/inference_target"
cp "$training_log/policy_traced.pt" "$training_log/cfgs.yaml" "$training_log/inference_target/"
```

Go2 は `dump_training_data.py` を使います。コントローラは次の固定ディレクトリから `policy_traced.pt` と `cfgs.yaml` を読み込みます。

| 対象 | ディレクトリ |
| --- | --- |
| Go2 | `~/genesis_ws/logs/go2-walking/inference_target/` |
| 二脚 | `~/genesis_ws/sandbox/genesis_simple_bipedal_rl/logs/simple_bipedal_walking/inference_target/` |

C++ の単体サンプルは次のように実行できます。

```bash
rosrun inference_tutorial inference /absolute/path/to/policy_traced.pt
```

このサンプルの入力は `1 × 45` の乱数です。観測次元が異なるモデルでは `inference.cpp` の `obs_dim` を合わせて再ビルドしてください。

### その他の教材

- `choreonoid_tutorial/config/`: Body / URDF ファイルの説明と変換例。
- `choreonoid_tutorial/logs/`、`videos/`: シミュレーションのログと動画。
- `choreonoid_tutorial/scripts/export_video.py`: `pictures/<名前>%08d.png` を FFmpeg で動画化。成功後に該当 PNG を削除します。実行例は `python3 src/lecture2025/choreonoid_tutorial/scripts/export_video.py -n scene` です。
