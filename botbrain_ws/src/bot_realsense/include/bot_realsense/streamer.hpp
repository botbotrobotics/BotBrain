#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/logger.hpp>

namespace bot_realsense {

struct StreamConfig {
  std::string mount_base;
  int rtsp_port{};
  std::string encoder;  // "auto", "nvenc" or "x264"
  int color_w{}, color_h{}, color_fps_in{}, color_fps_out{};
  int color_bitrate{};
  int depth_w{}, depth_h{}, depth_fps_in{}, depth_fps_out{};  // full-resolution depth; the stream is decimated 2x2
  int depth_bitrate{};
  double gop_s{};
  float depth_min_m{}, depth_max_m{};
};

// Serves rtsp://127.0.0.1:<rtsp_port>/<mount_base>/{color,depth} as H.264. Each mount's encoder runs only while an
// RTSP client is attached; all clients of a mount share one encoder.
class Streamer {
 public:
  // Builds the RTSP server and mounts, no encoding. Throws std::runtime_error on an invalid config or a missing encoder.
  Streamer(const rclcpp::Logger& logger, StreamConfig cfg);
  ~Streamer();
  Streamer(const Streamer&) = delete;
  Streamer& operator=(const Streamer&) = delete;
  Streamer(Streamer&&) = delete;
  Streamer& operator=(Streamer&&) = delete;
  // RTSP server on/off. start() throws std::runtime_error if the port cannot be bound. stop() disconnects all clients
  // and returns once their encoders have stopped (bounded at 5 s); call it while frames still flow.
  void start();
  void stop();

  bool color_wanted() const;
  bool depth_wanted() const;

  // Never block: frames are skipped down to the stream fps and dropped while the encoder is busy. Strides are in bytes,
  // stamp_ns is the capture time on the ROS clock. Call each from a single thread (colour and depth may differ).
  void push_color_yuyv(const uint8_t* data, int stride, uint64_t stamp_ns);
  void push_depth_z16(const uint16_t* data, int stride, float depth_scale_m, uint64_t stamp_ns);

  std::string encoder_name() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bot_realsense
