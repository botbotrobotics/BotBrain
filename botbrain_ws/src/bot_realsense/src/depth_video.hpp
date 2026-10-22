#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace bot_realsense {

// Raw Z16 sample -> grey luma: 0 = no depth, valid depth 16..235 on an inverse-sqrt scale of [min_m, max_m].
using GreyLut = std::array<uint8_t, 65536>;

void fill_grey_lut(GreyLut& lut, float depth_scale_m, float min_m, float max_m);

// librealsense decimation(2): the lower median of the block's non-zero samples, 0 if none.
inline uint16_t decimate_block(uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3) {
  const int n = (p0 != 0) + (p1 != 0) + (p2 != 0) + (p3 != 0);
  // Subtracting 1 wraps "no depth" to 0xFFFF so zeros sort last; the lower median of n valid ones is at (n-1)/2.
  const auto a = uint16_t(p0 - 1), b = uint16_t(p1 - 1), c = uint16_t(p2 - 1), d = uint16_t(p3 - 1);
  const uint16_t l0 = std::min(a, b), h0 = std::max(a, b), l1 = std::min(c, d), h1 = std::max(c, d);
  const uint16_t s0 = std::min(l0, l1), s1 = std::min(std::max(l0, l1), std::min(h0, h1));
  return uint16_t((n < 3 ? s0 : s1) + 1);
}

// Decimates a w x h Z16 image 2x2 and writes its (w/2) x (h/2) grey luma. w and h must be even.
void depth_to_grey(const uint16_t* z, size_t z_stride_bytes, int w, int h, const GreyLut& lut, uint8_t* luma,
                   size_t luma_stride);

}  // namespace bot_realsense
