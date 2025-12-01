# nav_cloning
## Running simulation

### 一括して起動
* nav_cloning (一定経路の模倣学習)
```
roscd nav_cloning/experiments/
./experiment_use_dl_output.sh
```
`nav_cloning/data`フォルダにログと学習済みモデルを保存  
シェルファイルのパラメータを変更することで様々な条件で実験可能

* nav_cloning_with_direction (経路選択を含む模倣学習)
```
roscd nav_cloning/experiments/
./experiment_with_direction_use_dl_output.sh
```
`nav_cloning/data`フォルダにログと学習済みモデルが保存  
シェルファイルのパラメータを変更することで様々な条件で実験可能

[![IMAGE](http://img.youtube.com/vi/6LG06ZbCjto/0.jpg)](https://youtu.be/6LG06ZbCjto)

### 分割して起動
* シミュレータの起動
```
roslaunch nav_cloning nav_cloning_sim.launch
```
* rviz上の2D Pose Estimateで自己位置を合わせる
* 実行
```
rosservice call /start_wp_nav
```
* save data:  /nav_cloning/data/result \
loss \
angle_error : navigationの出力と訓練されたモデルの出力の差 \
distance : 目標経路とロボットの位置の間の距離

## install
* 環境 ubuntu18.04, ros melodic

* ワークスペースの用意
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ../
catkin_make
```
* nav_cloningの用意
```
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/open-rdc/nav_cloning/master/nav_cloning.install
wstool init
wstool merge nav_cloning.install
wstool up
```
* 依存パッケージのインストール
```
cd ~/catkin_ws/src
rosdep init
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ../
catkin_make
```
* その他インストール
```
sudo apt install python-pip
pip install chainer==6.0
pip install scikit-image
```
## Docker
* Usage
example:
1. 起動
```
cd ~/catkin_ws/src/nav_cloning/docker
docker-compose up
```
or
```
docker pull -p 8080:80 masayaokada/nav_cloning:open-rdc
```
2. アクセス
Access to http://localhost:8080

### Data Analysis
https://github.com/open-rdc/nav_cloning/wiki



# nav_cloning

ROS (Noetic) 上で動作する、自律移動ロボット向け **視覚ベース経路追従・未知障害物回避** パッケージです。

本リポジトリでは、

- 3チャネル入力：RGB カメラ画像のみ
- 4チャネル入力：**RGB + Ground Mask（セマンティックセグメンテーション）**

の両方を扱える構成を想定しており、とくに 4ch 版では  
DeepLabV3+（mmsegmentation）による **地面領域マスク** を追加チャネルとして入力し、  
RGB のみでは困難だった未知障害物への汎化を狙います。

## 概要

- カメラ画像からの end-to-end ナビゲーション（模倣学習を想定）
- DeepLabV3+ + mmsegmentation による ground mask 推論
- 4チャネル入力（RGB + ground mask）対応ネットワーク
- シミュレータ／実機でのログ保存
  - 生 RGB 画像
  - 4ch 入力（チャンネルファイル）
- 学習・評価用データの保存（`.npy` / 画像ファイルなど）

---

## 動作環境

- OS: Ubuntu 20.04
- ROS: Noetic
- Python: 3.8 系
- GPU: NVIDIA RTX 系（推奨）
- 主なライブラリ（例）
  - OpenCV (Python)
  - PyTorch
  - mmcv / mmsegmentation
  - NumPy, scikit-image など

※ 実際のバージョンは自分の環境に合わせて調整してください。

---

## インストールとビルド

### 1. リポジトリのクローン

```bash
cd ~/catkin_ws/src
git clone https://github.com/21C1072takahashishoku/nav_cloning.git
cd ~/catkin_ws
catkin_make         # または catkin build
````

### 2. Python 依存パッケージ
mmsegmentation / mmcv 等は、mmsegmentation 公式のインストール手順に従って導入してください。

## ディレクトリ構成（抜粋）

```text
nav_cloning/
  ├── CMakeLists.txt
  ├── package.xml
  ├── src/
  │   ├── navcloning_node.py          # 3ch 版ノード（例）
  │   ├── navcloning_4ch_node.py      # 4ch 版ノード（例）
  │   └── nav_cloning_4ch_net.py      # 4ch 対応ネットワーク
  ├── launch/
  │   ├── nav_cloning_3ch.launch      # 3ch 版起動用（例）
  │   ├── nav_cloning_4ch.launch      # 4ch 版起動用
  │   └── mmseg_groundmask.launch     # ground mask 推論用
  ├── mmseg/
  │   ├── configs/
  │   ├── checkpoints/                # DeepLabV3+ 等の学習済みモデル（Git 管理外）
  │   └── ...
  ├── data/
  ├── README.md
  └── .gitignore
```

`.gitignore` の例：

```gitignore
data/
eigen/
mmseg/checkpoints/
```

* `mmseg/checkpoints/` 以下の `.pth` など大容量ファイルは Git 管理外としています。
* `data/` もログ・学習データ用ディレクトリとして Git 管理外としています。

---

## 大容量ファイルとチェックポイントの扱い

GitHub のファイルサイズ制限（100MB）により、DeepLabV3+ などの `.pth` はリポジトリに含めていません。
各自でダウンロードし、以下のように配置してください。

```text
nav_cloning/
  mmseg/
    checkpoints/
      deeplabv3_r101-d8_512x1024_80k_cityscapes_20200606_113503-9e428899.pth
      deeplabv3_r101-d8_512x512_160k_ade20k_20200615_105816-b1f72b3b.pth
```

* 取得元：mmsegmentation 公式配布ページなど
* `.gitignore` により `mmseg/checkpoints/` は Git にコミットされません。

---

## 4チャネル入力（RGB + Ground Mask）の流れ

### 入力構成

* RGB 画像：`(H, W, 3)` （0〜255 の uint8）
* Ground Mask：`(H, W, 1)`

  * 地面 = 1（または 255）、非地面 = 0 の 2値マスクを想定
* モデルへの入力：

  * `(H, W, 4)` = `[R, G, B, ground_mask]` を作成し、`(C, H, W)` に変形してネットワークへ入力

Ground Mask は DeepLabV3+（mmsegmentation）により、

* road
* sidewalk
* terrain

などを「地面クラス」としてマージして 1 チャネル化したものを想定しています。


## 1. 4ch ナビゲーションノード

4ch 入力対応の nav_cloning ノードを起動します。

```bash
roslaunch nav_cloning nav_cloning_4ch.launch
```

`nav_cloning_4ch.launch`（例）では、以下を想定します。

* ノード：`navcloning_4ch_node.py`
* 購読：

  * `/camera/front/image_raw` (RGB)
  * `/nav_cloning/ground_mask` (mono8)
* 発行：

  * `/cmd_vel` (`geometry_msgs/Twist`)

ノード内部での 4ch 生成処理イメージ：

```python
# RGB: (H, W, 3), uint8
# mask: (H, W), uint8 (0 or 255)

rgb = rgb_image.astype(np.float32) / 255.0       # 0〜1 に正規化
mask = (mask_image > 0).astype(np.float32)      # 0 or 1 に変換
mask = mask[:, :, None]                         # (H, W, 1) に変形

input_4ch = np.concatenate([rgb, mask], axis=2) # (H, W, 4)

# PyTorch テンソルに変換してモデルへ
tensor = torch.from_numpy(input_4ch).permute(2, 0, 1).unsqueeze(0).to(device)
action = self.policy(tensor)  # 速度コマンド推論
```

