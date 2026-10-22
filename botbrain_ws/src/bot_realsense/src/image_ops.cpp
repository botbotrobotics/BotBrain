#include "bot_realsense/image_ops.hpp"

#include <librealsense2/rsutil.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <string>

namespace bot_realsense {
namespace {

uint8_t clamp8(int v) { return static_cast<uint8_t>(v < 0 ? 0 : v > 255 ? 255 : v); }

const uint16_t* row_of(const uint16_t* image, int stride_bytes, int y) {
  return reinterpret_cast<const uint16_t*>(reinterpret_cast<const uint8_t*>(image) + size_t(y) * stride_bytes);
}

bool is_millimetres(float depth_scale_m) { return std::fabs(depth_scale_m - 0.001f) < 1e-9f; }

uint16_t to_mm(uint16_t z, float mm_per_unit) {
  return static_cast<uint16_t>(std::min(65535.f, static_cast<float>(z) * mm_per_unit + 0.5f));
}

}  // namespace

RateLimiter::RateLimiter(double hz) : period_ns_(hz > 0 ? static_cast<int64_t>(1e9 / hz) : 0) {}

bool RateLimiter::allow(int64_t stamp_ns) {
  if (period_ns_ <= 0) return true;
  const int64_t early = next_ns_ - stamp_ns;
  // Up to a tenth of a period early still takes the slot, so frame jitter does not push the pick a frame later.
  if (early > period_ns_ / 10 && early <= 2 * period_ns_) return false;
  const bool resync = early < -period_ns_ || early > 2 * period_ns_;
  next_ns_ = resync ? stamp_ns + period_ns_ : next_ns_ + period_ns_;
  return true;
}

void yuyv_to_rgb8(const uint8_t* yuyv, int stride_bytes, int width, int height, uint8_t* rgb) {
  for (int y = 0; y < height; ++y) {
    const uint8_t* s = yuyv + size_t(y) * stride_bytes;
    uint8_t* d = rgb + size_t(y) * width * 3;
    for (int x = 0; x < width; x += 2, s += 4, d += 6) {
      const int c0 = s[0] - 16, u = s[1] - 128, c1 = s[2] - 16, v = s[3] - 128;
      const int r = 409 * v + 128, g = -100 * u - 208 * v + 128, b = 516 * u + 128;
      d[0] = clamp8((298 * c0 + r) >> 8);
      d[1] = clamp8((298 * c0 + g) >> 8);
      d[2] = clamp8((298 * c0 + b) >> 8);
      d[3] = clamp8((298 * c1 + r) >> 8);
      d[4] = clamp8((298 * c1 + g) >> 8);
      d[5] = clamp8((298 * c1 + b) >> 8);
    }
  }
}

void depth_to_mm(const uint16_t* z16, int stride_bytes, int width, int height, float depth_scale_m, uint16_t* mm) {
  const bool copy = is_millimetres(depth_scale_m);
  const float mm_per_unit = depth_scale_m * 1000.f;
  for (int y = 0; y < height; ++y) {
    const uint16_t* in = row_of(z16, stride_bytes, y);
    uint16_t* out = mm + size_t(y) * width;
    if (copy) {
      std::memcpy(out, in, size_t(width) * sizeof(uint16_t));
    } else {
      for (int x = 0; x < width; ++x) out[x] = to_mm(in[x], mm_per_unit);
    }
  }
}

DepthAligner::DepthAligner(const rs2_intrinsics& depth, const rs2_extrinsics& depth_to_color,
                           const rs2_intrinsics& color)
    : depth_w_(depth.width),
      depth_h_(depth.height),
      color_(color),
      translation_{depth_to_color.translation[0], depth_to_color.translation[1], depth_to_color.translation[2]},
      x0_(size_t(depth.width)),
      y0_(size_t(depth.width)),
      x1_(size_t(depth.width)),
      y1_(size_t(depth.width)) {
  switch (color.model) {
    case RS2_DISTORTION_NONE:
      std::fill(std::begin(color_.coeffs), std::end(color_.coeffs), 0.f);
      break;
    case RS2_DISTORTION_BROWN_CONRADY:
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      scaled_tangential_ = true;
      break;
    default:
      throw std::invalid_argument(std::string("cannot align to a colour lens with ") +
                                  rs2_distortion_to_string(color.model) + " distortion");
  }

  const size_t corners = size_t(depth_w_ + 1) * size_t(depth_h_ + 1);
  ray_x_.resize(corners);
  ray_y_.resize(corners);
  ray_z_.resize(corners);
  const float* r = depth_to_color.rotation;  // column-major
  size_t i = 0;
  for (int y = 0; y <= depth_h_; ++y) {
    for (int x = 0; x <= depth_w_; ++x, ++i) {
      const float corner[2] = {static_cast<float>(x) - 0.5f, static_cast<float>(y) - 0.5f};
      float p[3];
      rs2_deproject_pixel_to_point(p, &depth, corner, 1.0f);
      ray_x_[i] = r[0] * p[0] + r[3] * p[1] + r[6] * p[2];
      ray_y_[i] = r[1] * p[0] + r[4] * p[1] + r[7] * p[2];
      ray_z_[i] = r[2] * p[0] + r[5] * p[1] + r[8] * p[2];
    }
  }
}

void DepthAligner::project_row(const uint16_t* row, size_t top, size_t bottom, float depth_scale_m) {
  const float fx = color_.fx, fy = color_.fy, ppx = color_.ppx, ppy = color_.ppy;
  const float k1 = color_.coeffs[0], k2 = color_.coeffs[1], p1 = color_.coeffs[2], p2 = color_.coeffs[3],
              k3 = color_.coeffs[4];
  const float max_u = static_cast<float>(color_.width + 1), max_v = static_cast<float>(color_.height + 1);
  const float tx = translation_[0], ty = translation_[1], tz = translation_[2];
  const bool scaled = scaled_tangential_;
  // rs2_project_point_to_pixel for the Brown-Conrady family, rounded like rs2::align. Clamping keeps the int
  // conversion defined (zero depth gives garbage that is never used); outside stays outside after clipping.
  const auto pixel = [=](float rx, float ry, float rz, float z, int& u, int& v) {
    const float inv = 1.f / (z * rz + tz);
    const float x = (z * rx + tx) * inv, y = (z * ry + ty) * inv;
    const float r2 = x * x + y * y;
    const float f = 1.f + r2 * (k1 + r2 * (k2 + r2 * k3));
    const float sx = scaled ? x * f : x, sy = scaled ? y * f : y;
    const float dx = x * f + 2.f * p1 * sx * sy + p2 * (r2 + 2.f * sx * sx);
    const float dy = y * f + 2.f * p2 * sx * sy + p1 * (r2 + 2.f * sy * sy);
    // rs2::align truncates x + 0.5 rather than rounding (std::lround differs just left of 0 and at exact halves);
    // truncate the same way so the footprints match it pixel for pixel.
    u = static_cast<int>(std::trunc(std::fmin(std::fmax(dx * fx + ppx, -2.f), max_u) + 0.5f));
    v = static_cast<int>(std::trunc(std::fmin(std::fmax(dy * fy + ppy, -2.f), max_v) + 0.5f));
  };
  const float *rx = ray_x_.data() + top, *ry = ray_y_.data() + top, *rz = ray_z_.data() + top;
  const float *bx = ray_x_.data() + bottom + 1, *by = ray_y_.data() + bottom + 1, *bz = ray_z_.data() + bottom + 1;
  int *x0 = x0_.data(), *y0 = y0_.data(), *x1 = x1_.data(), *y1 = y1_.data();
  const int w = depth_w_;  // a local bound: the int stores below could otherwise alias the member
  for (int x = 0; x < w; ++x) {
    const float z = depth_scale_m * static_cast<float>(row[x]);
    pixel(rx[x], ry[x], rz[x], z, x0[x], y0[x]);
    pixel(bx[x], by[x], bz[x], z, x1[x], y1[x]);
  }
}

void DepthAligner::align(const uint16_t* z16, int stride_bytes, float depth_scale_m, uint16_t* out) {
  const int cw = color_.width, ch = color_.height;
  std::fill(out, out + size_t(cw) * size_t(ch), uint16_t{0});
  const bool copy = is_millimetres(depth_scale_m);
  const float mm_per_unit = depth_scale_m * 1000.f;
  const size_t corner_row = size_t(depth_w_) + 1;

  for (int y = 0; y < depth_h_; ++y) {
    const uint16_t* row = row_of(z16, stride_bytes, y);
    project_row(row, size_t(y) * corner_row, size_t(y + 1) * corner_row, depth_scale_m);
    for (int x = 0; x < depth_w_; ++x) {
      const uint16_t z = row[x];
      if (!z) continue;
      // Clip footprints at the border like rs2::align's SSE/NEON paths (the generic one drops them whole).
      const int u0 = std::max(x0_[size_t(x)], 0), v0 = std::max(y0_[size_t(x)], 0);
      const int u1 = std::min(x1_[size_t(x)], cw - 1), v1 = std::min(y1_[size_t(x)], ch - 1);
      const uint16_t value = copy ? z : to_mm(z, mm_per_unit);
      for (int v = v0; v <= v1; ++v) {
        uint16_t* o = out + size_t(v) * size_t(cw);
        for (int u = u0; u <= u1; ++u) {
          if (!o[u] || value < o[u]) o[u] = value;
        }
      }
    }
  }
}

}  // namespace bot_realsense
