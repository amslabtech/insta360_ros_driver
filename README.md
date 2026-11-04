# insta360_ros_driver

> ## 🔧 amsl による追記・改変について
> 本リポジトリは、オープンソースの [ai4ce/insta360_ros_driver](https://github.com/ai4ce/insta360_ros_driver) を
> **amsl (amslabtech)** がフォークし、機能追加・改変したものです。**元の ai4ce の機能はすべて残しています。**
>
> ### amsl が追加した機能
> - **統合ノード `insta360_all_node`**：カメラを **1回だけ開き**、以下の **3トピックを同時配信**します。
>   - `/dual_fisheye/image/compressed`（生のデュアル魚眼 H.264 **または H.265**）
>   - `/imu/data_raw_insta360`
>     （`/imu/data_raw` にしないこと。`whill_humble/all.launch.py` では Xsens が使用中で衝突する）
>   - **`/insta360/panorama/compressed`**（リアルタイム・パノラマ、JPEG）
> - パノラマは Insta360 **MediaSDK の `RealTimeStitcher`**（`DYNAMICSTITCH`）で生成。
>   SDK 付属デモ `realtime_stitcher_demo.cc` と**同じ方法をそのまま使用**しています。
>   FlowState は既定で **off**（`enable_flowstate: false`）＝カメラ本体固定のパノラマで、移動ロボット向け。
> - **撮影時刻スタンプ**（`use_sdk_timestamp`）：`header.stamp` をカメラの**撮影時刻**にするため、
>   パノラマのステッチ／エンコードに遅延があっても、rosbag 内で IMU と画像の時刻がズレません。
>   `decoder` が出す `/dual_fisheye/image` にも同じ撮影時刻が引き継がれます。
> - `decoder` は受信メッセージの `format`（`"h264"` / `"h265"`）を見て codec を**自動判定**します。
> - `bringup.launch.xml` は既定で**統合ノードと decoder**を起動します。
>   `equirectangular` / `imu_filter` は**コメントアウト**してあり、外せば従来どおり
>   `/equirectangular/image`・`/imu/data` も得られます。
>   （※ カメラを自前で開くノードは、統合ノードと**同時起動するとカメラ競合で失敗**します）
> - 設定は **`config/panorama.yaml`**（出力解像度・stitch 種別・crop・publish_fps・live_resolution・撮影時刻スタンプ等）。
>
> ### SDK の導入（amsl 改変版で必要）
> **Insta360 X6 対応のため CameraSDK 2.1.8 / MediaSDK 3.1.5 を使用します。**
> 旧 SDK とは非互換で、混在させるとビルドできません（`PreviewParam` のAPI変更、
> `SyncLocalTimeToCamera` のシグネチャ変更など）。
>
> - **CameraSDK 2.1.8**：`camera` / `stream` ヘッダを `include/` に、`libCameraSDK.so`(17MB) を `lib/` に配置。
> - **MediaSDK 3.1.5**：`MediaSDK-3.1.5-linux-amd64.deb` を Insta360 SDK から入手し
>   `sudo dpkg -i` で導入（`/opt/MediaSDK-3.1.5-linux/` に入る）。依存に `libopenblas0` が必要。
>   さらに**ヘッダ `ins_*.h` を `include/media/` にも配置**します
>   （システムに旧 MediaSDK の `/usr/include/ins_*.h` が残っていても確実に新版を使うため）。
> - **SDK バイナリはリポジトリに含めません**（`.gitignore` 済）。**各自でダウンロードして配置**してください。
>
> #### 実装上の重要な制約
> - **`insta360_all_node` に OpenCV をリンクしてはいけない。**
>   `libMediaSDK.so` が OpenCV 4.7（`libopencv_world.so.407`, 2.2GB）を同梱しており、
>   システム OpenCV を同一プロセスに入れると `main()` 到達前に SIGSEGV します。
>   そのため JPEG 化は `cv::imencode` ではなく **libjpeg 直叩き**（RGBA→JPEG）で行っています。
>   `decoder` / `equirectangular_cpp` は MediaSDK をリンクしないので OpenCV を使って構いません。
> - **RPATH の順序が重要**：`/opt/MediaSDK-3.1.5-linux/lib` にも別ビルドの `libCameraSDK.so`(20MB) が
>   あるため、同梱版(17MB)を先に見せています（`CMakeLists.txt` の `INSTA360_RPATH`）。
> - 導入後は `ldd build/insta360_ros_driver/insta360_all_node` で
>   `libMediaSDK.so => /opt/MediaSDK-3.1.5-linux/lib/...`、
>   `libCameraSDK.so => <このパッケージ>/lib/...`、
>   システム OpenCV が**含まれない**ことを確認してください。
> - **連続再起動時の注意**：直前セッション終了直後はカメラが USB 上に再列挙されるまで数秒かかります。
>   その間に探索すると空リストが返るだけでなく、SDK 内部スレッドで `std::bad_function_call` が
>   投げられてプロセスごと落ちることがあります（別スレッドなので `main` の `try/catch` では
>   捕まえられません）。このため `open_initial_delay`（既定 2.0 秒）だけ待ってから探索し、
>   見つからなければ `open_retry_count` 回リトライします。実測で間隔ゼロの連続再起動 4/4 成功。
> - **GPU について**：ステッチャの blender は Vulkan を使います。コンテナ環境では NVIDIA の
>   ドライバ本体が注入されていてもローダ用マニフェストが無いことがあり、その場合
>   `vulkan error -9 ERROR_INCOMPATIBLE_DRIVER` が出て CPU にフォールバックします。
>   `/usr/share/vulkan/icd.d/nvidia_icd.json`（`library_path: libGLX_nvidia.so.0`）と
>   `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`（`libEGL_nvidia.so.0`）を置けば解消します。
>   なお映像デコード（NVDEC）と CUDA 演算はこれとは独立に最初から効いています。
>
> ---
> 以下は元の ai4ce のドキュメント（英語）です。

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/ai4ce/insta360_ros_driver/blob/79588d9e0e9d029c3371d4095ea718daaf1e06fb/src/main.cpp#L126) to edit the resolution.

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

> **amsl 注記**: 現在この実機構成は **CameraSDK 2.1.8 / MediaSDK 3.1.5**（Insta360 X6 対応）に固定されています。
> 上記より古い SDK ではビルドできません。詳細は冒頭の「SDK の導入」を参照。

```
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Before continuing, **make sure the camera is set to dual-lens mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Usage
The camera provides images natively in H264 compressed image format. We have a decoder node that 

### Camera Bringup
The camera can be brought up with the following launch file
```
ros2 launch insta360_ros_driver bringup.launch.xml
```
![bringup](docs/bringup_rqt.png)

A dual fisheye image will be published.

![dual_fisheye](docs/dual_fisheye.png)

#### Published Topics
既定の `bringup.launch.xml`（統合ノード + decoder）で実際に出るのは以下の4つです。

| トピック | 型 | 備考 |
|---|---|---|
| `/insta360/panorama/compressed` | `sensor_msgs/CompressedImage` | `format="jpeg"`。MediaSDK でステッチしたパノラマ |
| `/dual_fisheye/image/compressed` | `sensor_msgs/CompressedImage` | `format="h264"` または `"h265"`（再エンコードなしの生ビットストリーム） |
| `/dual_fisheye/image` | `sensor_msgs/Image` | `bgr8`。decoder が上記をデコードしたもの |
| `/imu/data_raw_insta360` | `sensor_msgs/Imu` | `orientation_covariance[0] = -1`（姿勢推定なし） |

`/equirectangular/image` と `/imu/data` は、`bringup.launch.xml` で該当ノードの
コメントアウトを外したときのみ出ます。

The launch file has the following optional arguments:
- equirectangular (default="false")

This publishes equirectangular images. You can configure these parameters in `config/equirectangular.yaml`.
![equirectangular](docs/equirectangular.png)

- imu_filter (default="true")

This uses the [imu_filter_madgwick](https://wiki.ros.org/imu_filter_madgwick) package to approximate orientation from the IMU. Note that by default, we publish `/imu/data_raw_insta360` (amsl 改変版。上流は `/imu/data_raw`) which only contains linear acceleration and angular velocity.
madgwick フィルタを使う場合は `config/imu_filter.yaml` 側の入力トピックを合わせること。 The madgwick filter uses this information to publish orientation to `/imu/data`. You can configure the filter in `config/imu_filter.yaml`. 

![IMU](https://github.com/user-attachments/assets/02b50cad-8415-4dde-9014-9ab3a4d415b9)

## Equirectangular Calibration
You can adjust the extrinsic parameters used to improve the equirectangular image. 
```
# Run the camera driver
ros2 run insta360_ros_driver insta360_ros_driver
# Activate image decoding
ros2 run insta360_ros_driver decoder
# Run the equirectangular node in calibration mode
ros2 run insta360_ros_driver equirectangular.py --calibrate
```
This will open an app to adjust the extrinsics. You can press 's' to get the parameters in YAML format.
![Equirectangular Calibration](docs/calibration.png)

Pressing 's' will return the parameters via the terminal. You can copy paste this onto the configuration file as needed. By default, the launch file reads this from `config/equirectangular.yaml`

```
==================================================
CALIBRATION PARAMETERS (YAML FORMAT)
==================================================
equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 960
    translation: [0.0, 0.0, -0.105]
    rotation_deg: [-0.5, 0.0, 1.1]
    gpu: True
    out_width: 1920
    out_height: 960
==================================================
```

Note that decode.py will most likely drop frames depending on your system. If you do not care about live processing, you can simply record the `/dual_fisheye/image/compressed` topic and decompress it later after recording.
```
ros2 bag record /dual_fisheye/image /imu/data_raw_insta360
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)
