#pragma once

#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "bot_realsense/image_ops.hpp"
#include "bot_realsense/latest_slot.hpp"
#include "bot_realsense/streamer.hpp"
#include "bot_realsense/timing.hpp"

namespace bot_realsense {

class RealsenseNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit RealsenseNode(const rclcpp::NodeOptions& options);
  ~RealsenseNode() override;
  RealsenseNode(const RealsenseNode&) = delete;
  RealsenseNode& operator=(const RealsenseNode&) = delete;
  RealsenseNode(RealsenseNode&&) = delete;
  RealsenseNode& operator=(RealsenseNode&&) = delete;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  struct Params {
    std::string serial_no, camera_name, tf_prefix;
    bool initial_reset{}, align_depth{}, stream_enable{}, auto_exposure_priority{};
    int depth_w{}, depth_h{}, depth_fps{}, color_w{}, color_h{}, color_fps{};
    double color_rate{}, depth_rate{}, frame_timeout_s{};
    StreamConfig stream;
  };

  struct ImageOutput {
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo info_msg;
    RateLimiter rate;
  };

  Params read_params();
  void open_device();
  void hardware_reset(const std::string& serial);
  void create_outputs(const rs2_intrinsics& depth, const rs2_intrinsics& color, const rs2_extrinsics& color_to_depth);
  ImageOutput make_output(const std::string& image_topic, const std::string& info_topic, const rs2_intrinsics& in,
                          const std::string& frame_id, const char* encoding, uint32_t bytes_per_pixel, double rate_hz);
  void start_capture();
  void stop_capture();
  void release();
  void depth_loop();
  void color_loop();
  void process_depth(const rs2::frame& frame, int64_t stamp_ns, const rs2::frame& previous, int64_t previous_ns);
  void process_color(const rs2::frame& frame, int64_t stamp_ns);
  int64_t frame_stamp_ns(const rs2::frame& frame);
  void watchdog_tick();
  bool camera_present() const;
  static bool wanted(const ImageOutput& out);
  template <class Fill>
  static void publish(ImageOutput& out, const rclcpp::Time& stamp, Fill&& fill);

  // Threads. Lifecycle callbacks and the watchdog timer run on the executor thread; they create and change every
  // member below, only while the workers are stopped: start_capture() starts them, stop_capture() joins them. While
  // active, librealsense's callbacks touch only the slots and its devices-changed callback only device_removed_. The
  // depth worker owns depth_out_ and aligned_out_, the colour worker color_out_; the colour worker hands aligned
  // depth its picks through aligned_pick_ns_ and only reads aligned_out_'s subscriber counts. Both workers only read
  // params_, depth_scale_ and aligner_, write their last_*_frame_ns_, and each makes its own push_* calls on streamer_.
  Params params_;
  std::string serial_;  // of the opened camera; kept across cleanup so recovery knows which camera to wait for
  std::optional<rs2::context> context_;
  rs2::device device_;
  rs2::sensor stereo_, rgb_;
  rs2::stream_profile depth_profile_, color_profile_;
  float depth_scale_ = 0.001f;
  int64_t depth_period_ns_ = 0;
  std::optional<DepthAligner> aligner_;
  std::unique_ptr<Streamer> streamer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  ImageOutput color_out_, depth_out_, aligned_out_;
  LatestSlot<rs2::frame> depth_slot_, color_slot_;
  std::thread depth_thread_, color_thread_;

  std::atomic<bool> device_removed_{false};
  std::atomic<int64_t> last_depth_frame_ns_{0}, last_color_frame_ns_{0};  // steady clock
  std::atomic<int64_t> aligned_pick_ns_{0};  // stamp of the colour frame aligned depth should match, 0 = none
  rclcpp::TimerBase::SharedPtr watchdog_;
  RetrySchedule retry_;
  bool waiting_logged_ = false;
};

}  // namespace bot_realsense
