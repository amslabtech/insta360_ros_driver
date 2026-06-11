// insta360_panorama_node.cpp
//
// insta360_ros_driver の raw H.264 / IMU 配信（旧 main.cpp）と、
// MediaSDK の RealTimeStitcher によるパノラマ配信（旧 insta360_panorama_driver）を
// 「カメラを1回だけ開く1ノード」に統合したもの。
//
// 配信トピック:
//   /dual_fisheye/image/compressed (sensor_msgs/CompressedImage, "h264")
//   /imu/data_raw                  (sensor_msgs/Imu)
//   <panorama_topic>               (sensor_msgs/CompressedImage, "jpeg")
//
// header.stamp は「撮影時刻」（SDKタイムスタンプ）を ROS 時刻へ換算して付与する。
// → パノラマの処理遅延が IMU との相対時刻ズレに化けない（後段の rosbag 整合のため）。

#include <algorithm>
#include <atomic>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ins_realtime_stitcher.h>
#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>
#include <opencv2/opencv.hpp>

class Insta360PanoramaNode;

// ===== 統合 StreamDelegate =====
// カメラの1ストリームを「raw H.264 / IMU 配信」と「stitcherへの供給」の両方へ分配する。
class MergedDelegate : public ins_camera::StreamDelegate {
public:
    using RawPub = rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr;
    using ImuPub = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;

    MergedDelegate(const std::shared_ptr<ins::RealTimeStitcher>& stitcher,
                   RawPub raw_pub, ImuPub imu_pub,
                   std::function<rclcpp::Time(int64_t)> sdk_to_ros)
        : stitcher_(stitcher), raw_pub_(std::move(raw_pub)),
          imu_pub_(std::move(imu_pub)), sdk_to_ros_(std::move(sdk_to_ros)) {}

    virtual ~MergedDelegate() = default;

    void OnAudioData(const uint8_t*, size_t, int64_t) override {}

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                     uint8_t streamType, int stream_index) override {
        // (1) stitcher へ供給（panorama由来）
        stitcher_->HandleVideoData(data, size, timestamp, streamType, stream_index);

        // (2) 生 H.264 を配信（main.cpp由来）。メインストリーム(index 0)のみ。
        if (stream_index == 0 && size > 0 && raw_pub_) {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header.stamp    = sdk_to_ros_(timestamp);   // 撮影時刻
            msg->header.frame_id = "camera_frame";
            msg->format          = "h264";
            msg->data.assign(data, data + size);
            raw_pub_->publish(std::move(msg));
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        // (1) stitcher へ供給（panorama由来）。ins_camera::GyroData と ins::GyroData は同レイアウト。
        std::vector<ins::GyroData> data_vec(data.size());
        std::memcpy(data_vec.data(), data.data(), data.size() * sizeof(ins_camera::GyroData));
        stitcher_->HandleGyroData(data_vec);

        // (2) IMU を配信（main.cpp由来）
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp    = sdk_to_ros_(gyro.timestamp);   // 取得時刻
            msg->header.frame_id = "imu_frame";
            msg->angular_velocity.x = gyro.gx;
            msg->angular_velocity.y = gyro.gy;
            msg->angular_velocity.z = gyro.gz;
            msg->linear_acceleration.x = gyro.ax * 9.80665;
            msg->linear_acceleration.y = gyro.ay * 9.80665;
            msg->linear_acceleration.z = gyro.az * 9.80665;
            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 1.0;                 // Neutral orientation
            msg->orientation_covariance[0] = -1.0;    // orientation 無し
            for (int i = 0; i < 9; i++) {
                msg->angular_velocity_covariance[i] = 0;
                msg->linear_acceleration_covariance[i] = 0;
            }
            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {
        ins::ExposureData ed{};
        ed.exposure_time = data.exposure_time;
        ed.timestamp     = data.timestamp;
        stitcher_->HandleExposureData(ed);
    }

private:
    std::shared_ptr<ins::RealTimeStitcher> stitcher_;
    RawPub raw_pub_;
    ImuPub imu_pub_;
    std::function<rclcpp::Time(int64_t)> sdk_to_ros_;
};

// ===== ノード本体 =====
class Insta360PanoramaNode : public rclcpp::Node {
public:
    Insta360PanoramaNode() : Node("insta360_node") {
        // --- パラメータ ---
        output_width_   = declare_parameter<int>("output_width", 960);
        output_height_  = declare_parameter<int>("output_height", 480);
        const std::string stitch_type = declare_parameter<std::string>("stitch_type", "dynamic");
        crop_top_       = declare_parameter<int>("crop_top", 0);
        crop_bottom_    = declare_parameter<int>("crop_bottom", 0);
        jpeg_quality_   = declare_parameter<int>("jpeg_quality", 90);
        const double publish_fps = declare_parameter<double>("publish_fps", 10.0);
        panorama_frame_id_ = declare_parameter<std::string>("panorama_frame_id", "insta360");
        const std::string panorama_topic =
            declare_parameter<std::string>("panorama_topic", "/insta360/panorama/compressed");
        const std::string live_resolution =
            declare_parameter<std::string>("live_resolution", "1440x720");
        use_sdk_timestamp_ = declare_parameter<bool>("use_sdk_timestamp", true);
        const std::string ts_unit = declare_parameter<std::string>("timestamp_unit", "us");

        // タイムスタンプ単位 → ns 係数
        if      (ts_unit == "ns") ns_per_unit_ = 1;
        else if (ts_unit == "ms") ns_per_unit_ = 1000000;
        else if (ts_unit == "us") ns_per_unit_ = 1000;
        else { RCLCPP_WARN(get_logger(), "timestamp_unit '%s' 不明 -> us", ts_unit.c_str()); ns_per_unit_ = 1000; }

        // 間引き: source(30fps) のうち stride に1枚 publish
        publish_stride_ = (publish_fps > 0.0)
            ? std::max(1, static_cast<int>(30.0 / publish_fps + 0.5))
            : 1;
        RCLCPP_INFO(get_logger(), "panorama publish stride=%d (~%.2f Hz), use_sdk_timestamp=%s, unit=%s",
                    publish_stride_, 30.0 / publish_stride_,
                    use_sdk_timestamp_ ? "true" : "false", ts_unit.c_str());

        // stitch_type
        ins::STITCH_TYPE st = ins::STITCH_TYPE::DYNAMICSTITCH;
        if      (stitch_type == "template") st = ins::STITCH_TYPE::TEMPLATE;
        else if (stitch_type == "optflow")  st = ins::STITCH_TYPE::OPTFLOW;
        else if (stitch_type == "dynamic")  st = ins::STITCH_TYPE::DYNAMICSTITCH;
        else RCLCPP_WARN(get_logger(), "unknown stitch_type '%s' -> dynamic", stitch_type.c_str());

        // --- Publishers ---
        raw_pub_  = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/dual_fisheye/image/compressed", rclcpp::QoS(10));
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        pano_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(panorama_topic, 10);

        // --- SDK 初期化・カメラオープン ---
        ins::InitEnv();
        ins_camera::SetLogLevel(ins_camera::LogLevel::WARNING);
        ins::SetLogLevel(ins::InsLogLevel::WARNING);

        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            discovery.FreeDeviceDescriptors(list);
            RCLCPP_ERROR(get_logger(), "No Insta360 device found.");
            throw std::runtime_error("No Insta360 device found.");
        }
        RCLCPP_INFO(get_logger(), "Found camera: %s (%s)",
                    list[0].camera_name.c_str(), list[0].serial_number.c_str());

        cam_ = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam_->Open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera.");
            discovery.FreeDeviceDescriptors(list);
            throw std::runtime_error("Failed to open camera.");
        }
        discovery.FreeDeviceDescriptors(list);  // Open() の後に解放
        RCLCPP_INFO(get_logger(), "Camera opened successfully.");

        // --- stitcher 設定（panorama挙動を移植）---
        stitcher_ = std::make_shared<ins::RealTimeStitcher>();
        ins::CameraInfo camera_info;
        auto preview_param = cam_->GetPreviewParam();
        camera_info.cameraName  = preview_param.camera_name;
        camera_info.decode_type = static_cast<ins::VideoDecodeType>(preview_param.encode_type);
        camera_info.offset      = preview_param.offset;
        const auto& crop = preview_param.crop_info;
        camera_info.window_crop_info_.crop_offset_x = crop.crop_offset_x;
        camera_info.window_crop_info_.crop_offset_y = crop.crop_offset_y;
        camera_info.window_crop_info_.dst_width     = crop.dst_width;
        camera_info.window_crop_info_.dst_height    = crop.dst_height;
        camera_info.window_crop_info_.src_width     = crop.src_width;
        camera_info.window_crop_info_.src_height    = crop.src_height;

        stitcher_->SetCameraInfo(camera_info);
        stitcher_->SetStitchType(st);
        stitcher_->EnableFlowState(true);
        stitcher_->SetOutputSize(output_width_, output_height_);
        stitcher_->SetStitchRealTimeDataCallback(
            [this](uint8_t* data[4], int linesize[4],
                   int width, int height, int format, int64_t timestamp) {
                try {
                    onStitchFrame(data, linesize, width, height, format, timestamp);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Exception in stitch callback: %s", e.what());
                } catch (...) {
                    RCLCPP_ERROR(get_logger(), "Unknown exception in stitch callback");
                }
            });

        // --- delegate（配信開始前にセット）---
        delegate_ = std::make_shared<MergedDelegate>(
            stitcher_, raw_pub_, imu_pub_,
            [this](int64_t ts) { return sdkToRos(ts); });
        cam_->SetStreamDelegate(delegate_);

        // --- 時刻同期（main.cpp由来。2引数 = utc, tzオフセット秒）---
        {
            time_t now_t = time(nullptr);
            std::tm tm{};
            localtime_r(&now_t, &tm);
            time_t utc_sec = timegm(&tm);
            cam_->SyncLocalTimeToCamera(static_cast<uint64_t>(now_t),
                                        static_cast<uint32_t>(utc_sec - now_t));
        }

        // --- ライブ配信開始 ---
        const ins_camera::VideoResolution live_res = mapResolution(live_resolution);
        ins_camera::LiveStreamParam param;
        param.video_resolution      = live_res;
        param.lrv_video_resulution  = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate         = 1024 * 1024 / 2;
        param.enable_audio          = false;
        param.using_lrv             = false;

        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            cam_->Close();
            throw std::runtime_error("Failed to start live streaming.");
        }
        stitcher_->StartStitch();
        is_running_ = true;
        RCLCPP_INFO(get_logger(),
            "Streaming started. Topics: /dual_fisheye/image/compressed, /imu/data_raw, %s",
            panorama_topic.c_str());
    }

    ~Insta360PanoramaNode() {
        if (is_running_) {
            cam_->StopLiveStreaming();
            stitcher_->CancelStitch();
        }
        if (cam_) cam_->Close();
    }

private:
    ins_camera::VideoResolution mapResolution(const std::string& s) {
        if (s == "3840x1920") return ins_camera::VideoResolution::RES_3840_1920P30;
        if (s == "2560x1280") return ins_camera::VideoResolution::RES_2560_1280P30;
        if (s == "1920x960")  return ins_camera::VideoResolution::RES_1920_960P30;
        if (s == "1440x720")  return ins_camera::VideoResolution::RES_1440_720P30;
        RCLCPP_WARN(get_logger(), "live_resolution '%s' 不明 -> 1440x720", s.c_str());
        return ins_camera::VideoResolution::RES_1440_720P30;
    }

    // SDK撮影タイムスタンプ → ROS時刻。3トピックで同一アンカー(ts0,ros0)を共有して相対時刻を保つ。
    rclcpp::Time sdkToRos(int64_t ts) {
        if (!use_sdk_timestamp_) {
            return now();
        }
        std::lock_guard<std::mutex> lk(anchor_mtx_);
        if (!anchor_set_) {
            anchor_ros_ = now();
            anchor_ts_  = ts;
            anchor_set_ = true;
        }
        const int64_t delta_ns = (ts - anchor_ts_) * ns_per_unit_;
        return anchor_ros_ + rclcpp::Duration(std::chrono::nanoseconds(delta_ns));
    }

    cv::Mat cropTopBottom(const cv::Mat& img) const {
        const int h = img.rows;
        const int top    = std::min(std::max(crop_top_, 0), h);
        const int bottom = std::min(std::max(crop_bottom_, 0), h - top);
        const int new_h  = h - top - bottom;
        if (new_h <= 0) return img;
        return img(cv::Rect(0, top, img.cols, new_h)).clone();
    }

    // stitch 出力コールバック。撮影時刻(timestamp)でスタンプし、間引いて panorama を配信。
    void onStitchFrame(uint8_t* data[4], int /*linesize*/[4],
                       int width, int height, int /*format*/, int64_t timestamp) {
        if (width <= 0 || height <= 0 || data[0] == nullptr) return;
        if (frame_count_++ % publish_stride_ != 0) return;

        cv::Mat rgba = cv::Mat(height, width, CV_8UC4, data[0]).clone();
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        cv::Mat out = cropTopBottom(bgr);

        std::vector<uint8_t> buf;
        const std::vector<int> enc = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imencode(".jpg", out, buf, enc);

        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp    = sdkToRos(timestamp);   // 入力フレームの撮影時刻が伝播
        msg.header.frame_id = panorama_frame_id_;
        msg.format          = "jpeg";
        msg.data            = std::move(buf);
        pano_pub_->publish(msg);
    }

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pano_pub_;

    // SDK
    std::shared_ptr<ins_camera::Camera>         cam_;
    std::shared_ptr<ins::RealTimeStitcher>      stitcher_;
    std::shared_ptr<ins_camera::StreamDelegate> delegate_;
    std::atomic<bool>                           is_running_{false};

    // パラメータ
    int output_width_{960};
    int output_height_{480};
    int crop_top_{0};
    int crop_bottom_{0};
    int jpeg_quality_{90};
    int publish_stride_{1};
    unsigned long frame_count_{0};
    std::string panorama_frame_id_{"insta360"};

    // タイムスタンプ換算
    bool use_sdk_timestamp_{true};
    int64_t ns_per_unit_{1000};
    std::mutex anchor_mtx_;
    bool anchor_set_{false};
    rclcpp::Time anchor_ros_;
    int64_t anchor_ts_{0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Insta360PanoramaNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("insta360_node"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
