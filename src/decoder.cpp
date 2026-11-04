#include <iostream>
#include <thread>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

static enum AVPixelFormat get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
    const enum AVPixelFormat *p;
    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_CUDA) {
            return *p;
        }
    }
    return AV_PIX_FMT_NONE;
}

class H264DecoderNode : public rclcpp::Node {
private:
    AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* pkt_ = nullptr;
    AVFrame* hw_frame_ = nullptr;
    AVFrame* sw_frame_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    cv::Mat bgr_frame_; 
    AVBufferRef *hw_device_ctx_ = nullptr;
    enum AVHWDeviceType hw_type_ = AV_HWDEVICE_TYPE_NONE;
    std::string format_;   // 初回メッセージで確定する "h264" / "h265"

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::thread publisher_thread_;
    // 撮影時刻を publisher スレッドまで運ぶ。now() で再スタンプすると
    // /dual_fisheye/image/compressed と時刻がずれ、bag の整合が取れなくなる。
    struct QueuedFrame {
        cv::Mat image;
        builtin_interfaces::msg::Time stamp;
    };
    std::queue<QueuedFrame> frame_publish_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_publisher_thread_{false};
    size_t max_queue_size_ = 10;
    
    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    std::string frame_id_ = "camera_frame";
    long dropped_frames_ = 0;

    // codec は最初のメッセージの format を見るまで分からないので遅延初期化する。
    // X4 は H.264 だが X6 は H.265 で配信しうるため、決め打ちにはできない。
    bool InitFFmpegDecoder(const std::string& format) {
        const bool hevc = (format == "h265");
        const AVCodecID sw_id = hevc ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264;
        const char* decoder_name = hevc ? "hevc_cuvid" : "h264_cuvid";

        hw_type_ = AV_HWDEVICE_TYPE_CUDA;
        codec_ = avcodec_find_decoder_by_name(decoder_name);
        if (!codec_) {
            RCLCPP_WARN(this->get_logger(), "Hardware decoder not available, falling back to software");
            hw_type_ = AV_HWDEVICE_TYPE_NONE;
            codec_ = avcodec_find_decoder(sw_id);
            if (!codec_) {
                RCLCPP_ERROR(this->get_logger(), "No %s decoder available", format.c_str());
                return false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Using hardware %s decoder (NVDEC)", format.c_str());
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            int err = av_hwdevice_ctx_create(&hw_device_ctx_, hw_type_, nullptr, nullptr, 0);
            if (err < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to create hardware device context, falling back to software");
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
                codec_ = avcodec_find_decoder(sw_id);
                if (!codec_) {
                    RCLCPP_ERROR(this->get_logger(), "No %s decoder available", format.c_str());
                    return false;
                }
            }
        }

        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            CleanupFFmpegDecoder();
            return false;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
            CleanupFFmpegDecoder();
            return false;
        }

        pkt_ = av_packet_alloc();
        if (!pkt_) {
            CleanupFFmpegDecoder();
            return false;
        }

        hw_frame_ = av_frame_alloc();
        if (!hw_frame_) {
            CleanupFFmpegDecoder();
            return false;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                CleanupFFmpegDecoder();
                return false;
            }
        }
        return true;
    }

    void PublisherThreadLoop() {
        while (!stop_publisher_thread_) {
            QueuedFrame frame_to_publish;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return !frame_publish_queue_.empty() || stop_publisher_thread_;
                });

                if (stop_publisher_thread_ && frame_publish_queue_.empty()) {
                    break;
                }
                if (frame_publish_queue_.empty()) {
                    continue;
                }
                frame_to_publish = frame_publish_queue_.front();
                frame_publish_queue_.pop();
            }

            if (!frame_to_publish.image.empty() && publisher_) {
                auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
                std_msgs::msg::Header header;
                header.stamp = frame_to_publish.stamp;  // 撮影時刻をそのまま引き継ぐ
                header.frame_id = frame_id_;
                cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, frame_to_publish.image);
                cv_image.toImageMsg(*img_msg);
                publisher_->publish(std::move(img_msg));
            }
        }
    }

    void DecodeAndDisplayPacket(AVPacket* packet, const builtin_interfaces::msg::Time& stamp) {
        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret < 0) {
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            } else if (ret < 0) {
                return;
            }

            // 間引き判定はデコード直後・色変換の前に行う（無駄な sws_scale を避ける）。
            // i_frame_only はかつてパーサの key_frame を見ていたが、パーサを廃止したので
            // デコード済みフレームのフラグで判定する（H.264/H.265 どちらでも同じに効く）。
            bool should_publish = true;
            if (i_frame_only_) {
                should_publish = (hw_frame_->key_frame == 1);
            } else if (skip_frame_ > 0) {
                should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
            }
            if (!should_publish) {
                av_frame_unref(hw_frame_);
                continue;
            }

            AVFrame* frame_to_display = hw_frame_;

            if (hw_frame_->format == AV_PIX_FMT_CUDA || hw_frame_->format == AV_PIX_FMT_VAAPI) {
                if (av_hwframe_transfer_data(sw_frame_, hw_frame_, 0) < 0) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
                frame_to_display = sw_frame_;
            }

            if (!sws_ctx_ && frame_to_display->width > 0 && frame_to_display->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_display->width, frame_to_display->height, (AVPixelFormat)frame_to_display->format,
                    frame_to_display->width, frame_to_display->height, AV_PIX_FMT_BGR24,
                    SWS_POINT, nullptr, nullptr, nullptr);
                
                if (!sws_ctx_) {
                    av_frame_unref(hw_frame_);
                    if (frame_to_display == sw_frame_) av_frame_unref(sw_frame_);
                    return; 
                }
                bgr_frame_.create(frame_to_display->height, frame_to_display->width, CV_8UC3);
            }

            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t* dst_data[4] = { bgr_frame_.data, nullptr, nullptr, nullptr };
                int dst_linesize[4] = { static_cast<int>(bgr_frame_.step[0]), 0, 0, 0 };

                sws_scale(sws_ctx_,
                            (const uint8_t* const*)frame_to_display->data, frame_to_display->linesize,
                            0, frame_to_display->height,
                            dst_data, dst_linesize);

                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    if (frame_publish_queue_.size() < max_queue_size_) {
                        frame_publish_queue_.push(QueuedFrame{bgr_frame_.clone(), stamp});
                    } else {
                        // publisher スレッドが追いつかずキューが溢れている。
                        // 無言で捨てると原因不明のfps低下に見えるので必ず知らせる。
                        ++dropped_frames_;
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                             "publish queue full: dropped %ld frames so far "
                                             "(decode is keeping up but publishing is not)",
                                             dropped_frames_);
                    }
                }
                queue_cv_.notify_one();
            }
            
            av_frame_unref(hw_frame_);
            if (frame_to_display == sw_frame_) {
                av_frame_unref(sw_frame_);
            }
        }
    }

    void CleanupFFmpegDecoder() {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (sw_frame_) {
            av_frame_free(&sw_frame_);
            sw_frame_ = nullptr;
        }
        if (hw_frame_) {
            av_frame_free(&hw_frame_);
            hw_frame_ = nullptr;
        }
        if (pkt_) {
            av_packet_free(&pkt_);
            pkt_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_close(codec_ctx_); 
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (hw_device_ctx_) {
            av_buffer_unref(&hw_device_ctx_);
            hw_device_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->format != "h264" && msg->format != "h265") {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Unsupported format '%s'", msg->format.c_str());
            return;
        }

        if (!codec_ctx_) {
            format_ = msg->format;
            if (!InitFFmpegDecoder(format_)) {
                CleanupFFmpegDecoder();
                return;
            }
        } else if (msg->format != format_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "format changed '%s' -> '%s' (restart required)",
                                 format_.c_str(), msg->format.c_str());
            return;
        }

        if (!pkt_ || !hw_frame_) {
            return;
        }

        // 1メッセージ = 1アクセスユニットなので、パーサを通さずそのまま渡す。
        // パーサを挟むとパラメータセットだけのパケットが分離され、cuvid が
        // "PPS id out of range" を大量に出力し続ける。
        pkt_->data = const_cast<uint8_t*>(msg->data.data());
        pkt_->size = static_cast<int>(msg->data.size());
        DecodeAndDisplayPacket(pkt_, msg->header.stamp);
        pkt_->data = nullptr;
        pkt_->size = 0;
    }

public:
    H264DecoderNode() : Node("h264_decoder_node") {
        this->declare_parameter("compressed_topic", "/dual_fisheye/image/compressed");
        this->declare_parameter("uncompressed_topic", "/dual_fisheye/image");
        this->declare_parameter("skip_frame", 0);
        this->declare_parameter("i_frame_only", false);
        this->declare_parameter("frame_id", "camera_frame");

        std::string subscribe_topic = this->get_parameter("compressed_topic").as_string();
        std::string publish_topic = this->get_parameter("uncompressed_topic").as_string();
        skip_frame_ = this->get_parameter("skip_frame").as_int();
        i_frame_only_ = this->get_parameter("i_frame_only").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            subscribe_topic, 10,
            std::bind(&H264DecoderNode::compressed_image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, 10);

        publisher_thread_ = std::thread(&H264DecoderNode::PublisherThreadLoop, this);
        
        // InitFFmpegDecoder() はここでは呼ばない。codec は最初のメッセージの
        // format ("h264" / "h265") を見るまで決まらないため、遅延初期化する。

        RCLCPP_INFO(this->get_logger(), "Dual-fisheye decoder node initialized (H.264/H.265)");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", subscribe_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", publish_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Skip frame: %d, I-frame only: %s", skip_frame_, i_frame_only_ ? "true" : "false");
    }

    ~H264DecoderNode() {
        stop_publisher_thread_ = true;
        queue_cv_.notify_one();
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
        CleanupFFmpegDecoder();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264DecoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
