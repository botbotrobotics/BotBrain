#include "depth_video.hpp"

#include <cmath>

namespace bot_realsense {

void fill_grey_lut(GreyLut& lut, float depth_scale_m, float min_m, float max_m) {
  const double near = 1.0 / std::sqrt(double(min_m)), far = 1.0 / std::sqrt(double(max_m));
  lut[0] = 0;
  for (size_t v = 1; v < lut.size(); ++v) {
    const double z = double(v) * depth_scale_m;
    const double t = std::clamp((near - 1.0 / std::sqrt(z)) / (near - far), 0.0, 1.0);
    lut[v] = uint8_t(16 + std::lround(219 * t));
  }
}

void depth_to_grey(const uint16_t* z, size_t z_stride_bytes, int w, int h, const GreyLut& lut, uint8_t* luma,
                   size_t luma_stride) {
  const auto* base = reinterpret_cast<const uint8_t*>(z);
  const auto out_w = size_t(w / 2), out_h = size_t(h / 2);
  for (size_t y = 0; y < out_h; ++y) {
    const auto* r0 = reinterpret_cast<const uint16_t*>(base + 2 * y * z_stride_bytes);
    const auto* r1 = reinterpret_cast<const uint16_t*>(base + (2 * y + 1) * z_stride_bytes);
    uint8_t* out = luma + y * luma_stride;
    for (size_t x = 0; x < out_w; ++x) {
      out[x] = lut[decimate_block(r0[2 * x], r0[2 * x + 1], r1[2 * x], r1[2 * x + 1])];
    }
  }
}

}  // namespace bot_realsense
