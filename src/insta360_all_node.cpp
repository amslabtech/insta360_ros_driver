// insta360_all_node.cpp
//
// カメラを1回だけ開き、raw H.264/H.265 + IMU + MediaSDK パノラマ を同時配信する統合ノード。
// USB 上カメラを開けるのは1プロセスだけなので、必要なものはすべてこのノードが出す。
//
// 配信トピック (いずれもパラメータで変更可):
//   /dual_fisheye/image/compressed (sensor_msgs/CompressedImage, "h264" または "h265")
//   /imu/data_raw_insta360         (sensor_msgs/Imu)
//   /insta360/panorama/compressed  (sensor_msgs/CompressedImage, "jpeg")
//
// CameraSDK 2.1.8 / MediaSDK 3.1.5 対応 (Insta360 X6)。
//
// OpenCV は使わない。libMediaSDK.so が OpenCV 4.7 (libopencv_world.so.407) を同梱しており、
// システム OpenCV を同一プロセスへ入れると main() 到達前に SIGSEGV する。
// 色変換は不要 (RGBA のまま JPEG 化)、クロップは行ポインタ、JPEG 化は libjpeg を直接使う。
#include <ins_realtime_stitcher.h>
#include <ins_stitcher.h>
#include <camera/camera.h>
#include <camera/device_discovery.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <jpeglib.h>
#include <csetjmp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct JpegErr {
    jpeg_error_mgr pub;
    jmp_buf jb;
};

void jpegErrorExit(j_common_ptr cinfo) {
    longjmp(reinterpret_cast<JpegErr*>(cinfo->err)->jb, 1);
}

void jpegNoOpMessage(j_common_ptr) {}

// SDK が返す RGBA バッファを、コピーなしでそのまま JPEG 化する。
// stride は linesize[0]（width*4 とは限らない）。row_offset/height でクロップを表現する。
bool encodeRgbaJpeg(const uint8_t* rgba, int stride, int width, int row_offset, int height,
                    int quality, std::vector<uint8_t>& out) {
    jpeg_compress_struct cinfo;
    JpegErr jerr;
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpegErrorExit;
    jerr.pub.output_message = jpegNoOpMessage;

    unsigned char* buf = nullptr;
    unsigned long len = 0;
    if (setjmp(jerr.jb)) {
        jpeg_destroy_compress(&cinfo);
        if (buf) free(buf);
        return false;
    }

    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &buf, &len);
    cinfo.image_width = static_cast<JDIMENSION>(width);
    cinfo.image_height = static_cast<JDIMENSION>(height);
    cinfo.input_components = 4;
    cinfo.in_color_space = JCS_EXT_RGBA;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = const_cast<JSAMPROW>(
            rgba + static_cast<size_t>(row_offset + cinfo.next_scanline) * stride);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    out.assign(buf, buf + len);
    free(buf);
    return true;
}

}  // namespace

// 1本のライブストリームを stitcher と ROS publisher へ同時に流し込む。
class MergedDelegate : public ins_camera::StreamDelegate {
public:
    using RawPub = rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr;
    using ImuPub = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;

    MergedDelegate(std::shared_ptr<ins::RealTimeStitcher> stitcher, bool feed_gyro,
                   RawPub raw_pub, std::string raw_format, std::string raw_frame_id,
                   ImuPub imu_pub, std::string imu_frame_id, double accel_scale,
                   std::function<rclcpp::Time(int64_t)> sdk_to_ros)
        : stitcher_(std::move(stitcher)), feed_gyro_(feed_gyro),
          raw_pub_(std::move(raw_pub)), raw_format_(std::move(raw_format)),
          raw_frame_id_(std::move(raw_frame_id)), imu_pub_(std::move(imu_pub)),
          imu_frame_id_(std::move(imu_frame_id)), accel_scale_(accel_scale),
          sdk_to_ros_(std::move(sdk_to_ros)) {}

    void OnAudioData(const uint8_t*, size_t, int64_t) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                     uint8_t stream_type, int stream_index) override {
        // (1) stitcher へ供給
        stitcher_->HandleVideoData(data, size, timestamp, stream_type, stream_index);

        // (2) 生ビットストリームを再エンコードせずそのまま配信。メインストリーム(index 0)のみ。
        if (stream_index == 0 && size > 0 && raw_pub_) {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header.stamp = sdk_to_ros_(timestamp);  // 撮影時刻
            msg->header.frame_id = raw_frame_id_;
            msg->format = raw_format_;
            msg->data.assign(data, data + size);
            raw_pub_->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        // ins_camera::GyroData と ins::GyroData は同レイアウト（公式デモも memcpy している）。
        if (feed_gyro_) {
            std::vector<ins::GyroData> v(data.size());
            std::memcpy(v.data(), data.data(), data.size() * sizeof(ins_camera::GyroData));
            stitcher_->HandleGyroData(v);
        }

        if (!imu_pub_) return;
        for (const auto& g : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = sdk_to_ros_(g.timestamp);
            msg->header.frame_id = imu_frame_id_;
            msg->angular_velocity.x = g.gx;
            msg->angular_velocity.y = g.gy;
            msg->angular_velocity.z = g.gz;
            msg->linear_acceleration.x = g.ax * accel_scale_;
            msg->linear_acceleration.y = g.ay * accel_scale_;
            msg->linear_acceleration.z = g.az * accel_scale_;
            msg->orientation.w = 1.0;
            msg->orientation_covariance[0] = -1.0;  // orientation 推定なし
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {
        ins::ExposureData ed{};
        ed.exposure_time = data.exposure_time;
        ed.timestamp = data.timestamp;
        stitcher_->HandleExposureData(ed);
    }

private:
    std::shared_ptr<ins::RealTimeStitcher> stitcher_;
    bool feed_gyro_;
    RawPub raw_pub_;
    std::string raw_format_;
    std::string raw_frame_id_;
    ImuPub imu_pub_;
    std::string imu_frame_id_;
    double accel_scale_;
    std::function<rclcpp::Time(int64_t)> sdk_to_ros_;
};

class Insta360AllNode : public rclcpp::Node {
public:
    Insta360AllNode() : Node("insta360_node") {
        output_width_ = declare_parameter<int>("output_width", 1280);
        output_height_ = declare_parameter<int>("output_height", 640);
        const std::string stitch_type = declare_parameter<std::string>("stitch_type", "dynamic");
        enable_flowstate_ = declare_parameter<bool>("enable_flowstate", false);
        const std::string model_dir =
            declare_parameter<std::string>("model_dir", "/opt/MediaSDK-3.1.5-linux/bin/models/");

        crop_top_ = declare_parameter<int>("crop_top", 0);
        crop_bottom_ = declare_parameter<int>("crop_bottom", 0);
        jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);
        const double publish_fps = declare_parameter<double>("publish_fps", 10.0);

        const std::string live_resolution = declare_parameter<std::string>("live_resolution", "auto");
        const int video_bitrate = declare_parameter<int>("video_bitrate", 1024 * 1024 / 2);

        const std::string panorama_topic =
            declare_parameter<std::string>("panorama_topic", "/insta360/panorama/compressed");
        panorama_frame_id_ = declare_parameter<std::string>("panorama_frame_id", "insta360");
        const bool publish_imu = declare_parameter<bool>("publish_imu", true);
        // 既定を /imu/data_raw にしないこと: whill_humble/all.launch.py では Xsens が
        // /imu/data_raw を使っており、衝突する。
        const std::string imu_topic =
            declare_parameter<std::string>("imu_topic", "imu/data_raw_insta360");
        const std::string imu_frame_id = declare_parameter<std::string>("imu_frame_id", "imu_frame");
        const double accel_scale = declare_parameter<double>("accel_scale", 9.80665);
        const bool publish_raw = declare_parameter<bool>("publish_raw", true);
        const std::string raw_topic =
            declare_parameter<std::string>("raw_topic", "/dual_fisheye/image/compressed");
        const std::string raw_frame_id =
            declare_parameter<std::string>("raw_frame_id", "camera_frame");

        // カメラのオープン再試行。直前セッション終了直後は USB 再列挙に数秒かかるため。
        open_initial_delay_ = declare_parameter<double>("open_initial_delay", 2.0);
        open_retry_count_ = declare_parameter<int>("open_retry_count", 15);
        open_retry_delay_ = declare_parameter<double>("open_retry_delay", 2.0);

        use_sdk_timestamp_ = declare_parameter<bool>("use_sdk_timestamp", true);
        // SDK のヘッダは「マイクロ秒」と書いているが、実測ではミリ秒だった
        // (X4/30fps で映像の連続stamp差が33、パノラマ10fpsで100)。ヘッダの記述を信用しないこと。
        const std::string ts_unit = declare_parameter<std::string>("timestamp_unit", "ms");
        if (ts_unit == "ns") ns_per_unit_ = 1;
        else if (ts_unit == "us") ns_per_unit_ = 1000;
        else if (ts_unit == "ms") ns_per_unit_ = 1000000;
        else RCLCPP_WARN(get_logger(), "timestamp_unit '%s' 不明 -> ms", ts_unit.c_str());

        pano_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(panorama_topic, 10);
        if (publish_raw) {
            raw_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(raw_topic, 10);
        }
        if (publish_imu) {
            imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
        }

        ins::InitEnv();
        if (!model_dir.empty()) ins::SetModelFileRootDir(model_dir);  // 末尾スラッシュ必須
        ins_camera::SetLogLevel(ins_camera::LogLevel::WARNING);
        ins::SetLogLevel(ins::InsLogLevel::WARNING);
        RCLCPP_INFO(get_logger(), "CameraSDK %s / MediaSDK %s",
                    ins_camera::GetSDKVersion().c_str(), ins::GetVersion().c_str());

        openCamera();

        auto preview = cam_->GetPreviewParam();  // move-only。コピーしないこと
        std::vector<std::string> calib;
        for (size_t i = 0; i < preview.GetCalibrationCount(); i++) {
            calib.push_back(preview.GetCalibration(i));
        }
        RCLCPP_INFO(get_logger(),
                    "preview: name='%s' encode=%s calib=%zu crop src=%ux%u dst=%ux%u offset=%d,%d",
                    preview.camera_name.c_str(),
                    preview.encode_type == ins_camera::VideoEncodeType::H265 ? "h265" : "h264",
                    calib.size(), preview.GetCropSrcWidth(), preview.GetCropSrcHeight(),
                    preview.GetCropDstWidth(), preview.GetCropDstHeight(),
                    preview.GetCropOffsetX(), preview.GetCropOffsetY());
        if (calib.empty()) {
            // 無音で壊れたパノラマを出さない。カメラがデュアルレンズモードか確認すること。
            throw std::runtime_error("キャリブレーションが取得できません（スティッチ不可）");
        }

        stitcher_ = std::make_shared<ins::RealTimeStitcher>();
        ins::CameraInfo info;
        info.cameraName = preview.camera_name;
        info.decode_type = static_cast<ins::VideoDecodeType>(preview.encode_type);
        info.SetCalibration(calib, preview.GetCropSrcWidth(), preview.GetCropSrcHeight(),
                            preview.GetCropDstWidth(), preview.GetCropDstHeight(),
                            preview.GetCropOffsetX(), preview.GetCropOffsetY());
        stitcher_->SetCameraInfo(info);

        ins::STITCH_TYPE st = ins::STITCH_TYPE::DYNAMICSTITCH;
        if (stitch_type == "template") st = ins::STITCH_TYPE::TEMPLATE;
        else if (stitch_type == "optflow") st = ins::STITCH_TYPE::OPTFLOW;
        else if (stitch_type != "dynamic") {
            RCLCPP_WARN(get_logger(), "stitch_type '%s' 不明 -> dynamic", stitch_type.c_str());
        }
        stitcher_->SetStitchType(st);
        // false でカメラ本体固定のパノラマになる（移動ロボット向け）。
        // MediaSDK 3.1.1 では false がクラッシュしたが 3.1.5 で修正済み。
        stitcher_->EnableFlowState(enable_flowstate_);
        stitcher_->SetOutputSize(output_width_, output_height_);
        stitcher_->SetStitchStateCallback([this](int err, const char* infostr) {
            RCLCPP_ERROR(get_logger(), "stitch error %d: %s", err, infostr ? infostr : "");
        });
        stitcher_->SetStitchRealTimeDataCallback(
            [this](uint8_t* data[4], int linesize[4], int w, int h, int fmt, int64_t ts) {
                try {
                    onStitchFrame(data, linesize, w, h, fmt, ts);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "stitch コールバックで例外: %s", e.what());
                } catch (...) {
                    RCLCPP_ERROR(get_logger(), "stitch コールバックで不明な例外");
                }
            });
        RCLCPP_INFO(get_logger(), "stitch=%s flowstate=%s output=%dx%d", stitch_type.c_str(),
                    enable_flowstate_ ? "on" : "off", output_width_, output_height_);

        // X6 は H.265 で配信しうる。decoder 側はこの format 文字列を見て codec を決める。
        const std::string raw_format =
            cam_->GetVideoEncodeType() == ins_camera::VideoEncodeType::H265 ? "h265" : "h264";
        delegate_ = std::make_shared<MergedDelegate>(
            stitcher_, enable_flowstate_, raw_pub_, raw_format, raw_frame_id, imu_pub_,
            imu_frame_id, accel_scale,
            [this](int64_t ts) { return sdkToRos(ts); });
        std::shared_ptr<ins_camera::StreamDelegate> base = delegate_;
        cam_->SetStreamDelegate(base);

        const auto utc = static_cast<uint64_t>(std::time(nullptr));
        cam_->SyncLocalTimeToCamera(utc, 0);

        // 間引き間隔は配信開始前に決めておく（コールバックが先に走っても未初期化にならないように）。
        publish_interval_ = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / std::max(publish_fps, 0.1)));

        startLiveStreaming(live_resolution, video_bitrate);
        stitcher_->StartStitch();
        is_running_ = true;

        RCLCPP_INFO(get_logger(), "streaming started. topics: %s (%.1f fps), %s (%s), %s",
                    panorama_topic.c_str(), publish_fps,
                    publish_raw ? raw_topic.c_str() : "(raw 無効)", raw_format.c_str(),
                    publish_imu ? imu_topic.c_str() : "(imu 無効)");
    }

    ~Insta360AllNode() override {
        if (is_running_) {
            cam_->StopLiveStreaming();
            stitcher_->CancelStitch();
        }
        if (cam_) cam_->Close();
    }

private:
    // 直前のセッションが終了した直後はカメラが USB 上に再列挙されるまで数秒かかる。
    // その間に GetAvailableDevices() を呼ぶと空リストが返るだけでなく、SDK 内部の
    // ワーカースレッドで std::bad_function_call が投げられてプロセスごと落ちることがある
    // （別スレッドなので main の try/catch では捕まえられない）。
    // bri の再起動時に必ず踏むため、最初の探索前に必ず待ってから、以降もリトライする。
    void openCamera() {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(open_initial_delay_)));

        for (int attempt = 1; attempt <= open_retry_count_; ++attempt) {
            ins_camera::DeviceDiscovery discovery;
            auto list = discovery.GetAvailableDevices();
            if (!list.empty()) {
                RCLCPP_INFO(get_logger(), "camera: %s (type=%d, serial=%s, fw=%s)",
                            list[0].camera_name.c_str(), static_cast<int>(list[0].camera_type),
                            list[0].serial_number.c_str(), list[0].fw_version.c_str());
                camera_type_ = list[0].camera_type;
                cam_ = std::make_shared<ins_camera::Camera>(list[0].info);
                const bool opened = cam_->Open();
                discovery.FreeDeviceDescriptors(list);  // Open() の後に解放（公式デモと同じ順序）
                if (opened) return;
                cam_.reset();
                RCLCPP_WARN(get_logger(), "カメラを開けません (%d/%d) — 再試行します",
                            attempt, open_retry_count_);
            } else {
                discovery.FreeDeviceDescriptors(list);
                RCLCPP_WARN(get_logger(), "カメラが見つかりません (%d/%d) — 再試行します",
                            attempt, open_retry_count_);
            }

            if (attempt < open_retry_count_ && rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(open_retry_delay_)));
            }
            if (!rclcpp::ok()) break;
        }
        throw std::runtime_error("カメラを開けませんでした（USB接続・デュアルレンズモード・"
                                 "USB Mode=Android を確認してください）");
    }

    // 機種分岐は持たない。カメラに対応解像度を問い合わせ、ダメなら候補を順に試す。
    // CameraSDK 2.1.8 のデモは SetVideoSubMode(VIDEO_LIVEVIEW) を呼ばなくなっているので、ここでも呼ばない。
    void startLiveStreaming(const std::string& requested, int bitrate) {
        std::vector<ins_camera::VideoResolution> candidates;

        const auto supported =
            cam_->GetSupportedVideoResolutions(ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM);
        for (const auto& r : supported) {
            RCLCPP_INFO(get_logger(), "supported live resolution: %s",
                        cam_->GetVideoResolutionName(r).c_str());
        }

        if (requested != "auto") {
            for (const auto& r : supported) {
                if (cam_->GetVideoResolutionName(r).rfind(requested, 0) == 0) candidates.push_back(r);
            }
            if (candidates.empty()) {
                RCLCPP_WARN(get_logger(),
                            "live_resolution '%s' は対応リストに無い -> 既定候補を試す",
                            requested.c_str());
            }
        } else {
            candidates = supported;
        }

        // 対応リストが空/不一致のときの保険。X6 は live_sub_mode が空のため、こちらが本線になりうる。
        for (auto r : {ins_camera::VideoResolution::RES_1920_960P30,
                       ins_camera::VideoResolution::RES_1440_720P30,
                       ins_camera::VideoResolution::RES_2560_1280P30,
                       ins_camera::VideoResolution::RES_3840_1920P30}) {
            if (std::find(candidates.begin(), candidates.end(), r) == candidates.end()) {
                candidates.push_back(r);
            }
        }

        for (auto r : candidates) {
            ins_camera::LiveStreamParam param;
            param.video_resolution = r;
            param.lrv_video_resulution = r;
            param.video_bitrate = static_cast<uint32_t>(bitrate);
            param.enable_audio = false;
            param.enable_gyro = true;
            param.using_lrv = false;
            if (cam_->StartLiveStreaming(param)) {
                RCLCPP_INFO(get_logger(), "live streaming started: %s",
                            cam_->GetVideoResolutionName(r).c_str());
                return;
            }
            RCLCPP_WARN(get_logger(), "StartLiveStreaming 失敗: %s",
                        cam_->GetVideoResolutionName(r).c_str());
        }
        cam_->Close();
        throw std::runtime_error("どの解像度でもライブ配信を開始できません");
    }

    // SDK撮影時刻 → ROS時刻。3トピックで同一アンカーを共有し相対時刻を保つ。
    rclcpp::Time captureToRos(int64_t ts) {
        std::lock_guard<std::mutex> lk(anchor_mtx_);
        if (!anchor_set_) {
            anchor_ros_ = now();
            anchor_ts_ = ts;
            anchor_set_ = true;
        }
        return anchor_ros_ + rclcpp::Duration(std::chrono::nanoseconds((ts - anchor_ts_) * ns_per_unit_));
    }

    rclcpp::Time sdkToRos(int64_t ts) { return use_sdk_timestamp_ ? captureToRos(ts) : now(); }

    void onStitchFrame(uint8_t* data[4], int linesize[4], int width, int height, int /*format*/,
                       int64_t timestamp) {
        if (!data || !data[0] || width <= 0 || height <= 0) return;
        // 間引きはエンコード前に。入力fpsは機種と解像度で変わる(X4:30, X6:約23)ので、
        // フレーム数ではなく経過時間で判定する。
        {
            const auto t = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lk(pub_mtx_);
            if (t < next_publish_) return;
            // 次回予定は実配信時刻ではなく理想グリッド上へ進める。実配信時刻を基準にすると
            // 入力周期との差が毎回積み上がり、目標fpsより遅くなる。
            next_publish_ += publish_interval_;
            if (next_publish_ < t) next_publish_ = t + publish_interval_;
        }

        const int top = std::min(std::max(crop_top_, 0), height);
        const int bottom = std::min(std::max(crop_bottom_, 0), height - top);
        const int cropped = height - top - bottom;
        if (cropped <= 0) return;

        std::vector<uint8_t> buf;
        if (!encodeRgbaJpeg(data[0], linesize[0], width, top, cropped, jpeg_quality_, buf)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "JPEG エンコード失敗");
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = sdkToRos(timestamp);  // 入力フレームの撮影時刻が伝播
        msg->header.frame_id = panorama_frame_id_;
        msg->format = "jpeg";
        msg->data = std::move(buf);
        pano_pub_->publish(std::move(msg));
    }

    std::shared_ptr<ins_camera::Camera> cam_;
    ins_camera::CameraType camera_type_{ins_camera::CameraType::Unknown};
    std::shared_ptr<ins::RealTimeStitcher> stitcher_;
    std::shared_ptr<MergedDelegate> delegate_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pano_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    int output_width_{1280};
    int output_height_{640};
    bool enable_flowstate_{false};
    int crop_top_{0};
    int crop_bottom_{0};
    int jpeg_quality_{90};
    std::mutex pub_mtx_;
    std::chrono::nanoseconds publish_interval_{0};
    std::chrono::steady_clock::time_point next_publish_{};
    std::string panorama_frame_id_{"insta360"};

    double open_initial_delay_{2.0};
    int open_retry_count_{15};
    double open_retry_delay_{2.0};

    bool use_sdk_timestamp_{true};
    int64_t ns_per_unit_{1000000};
    std::mutex anchor_mtx_;
    bool anchor_set_{false};
    rclcpp::Time anchor_ros_;
    int64_t anchor_ts_{0};

    bool is_running_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Insta360AllNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("insta360_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
