#pragma once

#include <librealsense2/h/rs_sensor.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace bot_realsense {

// Lets through at most `hz` frames per second, evenly spaced on the frame clock; hz <= 0 lets every frame through.
class RateLimiter {
 public:
  explicit RateLimiter(double hz = 0.0);
  bool allow(int64_t stamp_ns);

 private:
  int64_t period_ns_;
  int64_t next_ns_ = 0;
};

// BT.601 limited range with librealsense's integer maths, so colours match its own YUYV -> RGB8. Even width.
void yuyv_to_rgb8(const uint8_t* yuyv, int stride_bytes, int width, int height, uint8_t* rgb);

// Packed millimetre image from raw Z16 units of depth_scale_m metres.
void depth_to_mm(const uint16_t* z16, int stride_bytes, int width, int height, float depth_scale_m, uint16_t* mm);

// Depth registered into the colour image like rs2::align(RS2_STREAM_COLOR): each depth pixel fills the colour
// pixels between its projected corners, the nearest depth wins on overlap. Not thread-safe: align() reuses
// per-row scratch.
class DepthAligner {
 public:
  // Throws std::invalid_argument for a colour lens outside the Brown-Conrady family (D400 colour never is).
  DepthAligner(const rs2_intrinsics& depth, const rs2_extrinsics& depth_to_color, const rs2_intrinsics& color);
  // out: color.width x color.height packed, millimetres, 0 = no depth.
  void align(const uint16_t* z16, int stride_bytes, float depth_scale_m, uint16_t* out);

 private:
  // Colour pixel footprint of every pixel of one depth row into x0_..y1_; a separate loop so it vectorises.
  void project_row(const uint16_t* row, size_t top, size_t bottom, float depth_scale_m);

  int depth_w_, depth_h_;
  rs2_intrinsics color_;
  bool scaled_tangential_ = false;  // inverse/modified Brown-Conrady apply the tangential terms after radial
  float translation_[3];
  // Per depth pixel corner, (depth_w_ + 1) x (depth_h_ + 1): the deprojected ray at 1 m, rotated into colour axes.
  std::vector<float> ray_x_, ray_y_, ray_z_;
  std::vector<int> x0_, y0_, x1_, y1_;
};

}  // namespace bot_realsense
