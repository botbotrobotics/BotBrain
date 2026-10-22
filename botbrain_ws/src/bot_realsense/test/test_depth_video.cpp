#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "depth_video.hpp"

using bot_realsense::GreyLut;

namespace {

constexpr float kScale = 0.001f, kMin = 0.3f, kMax = 8.0f;

uint16_t reference_median(uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3) {
  std::vector<uint16_t> v;
  for (uint16_t p : {p0, p1, p2, p3})
    if (p) v.push_back(p);
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[(v.size() - 1) / 2];
}

// What the browser does with a decoded luma value (contract inverse); 0 = no depth.
double grey_to_metres(int y) {
  if (y < 8) return 0;
  const double t = (y - 16) / 219.0, near = 1 / std::sqrt(double(kMin)), far = 1 / std::sqrt(double(kMax));
  return std::pow(near - t * (near - far), -2);
}

GreyLut lut() {
  GreyLut l;
  bot_realsense::fill_grey_lut(l, kScale, kMin, kMax);
  return l;
}

}  // namespace

TEST(DepthVideo, DecimateBlockIsLowerMedianOfNonZero) {
  const uint16_t vals[] = {0, 1, 2, 7, 300, 301, 8000, 65534, 65535};
  for (uint16_t a : vals)
    for (uint16_t b : vals)
      for (uint16_t c : vals)
        for (uint16_t d : vals) ASSERT_EQ(bot_realsense::decimate_block(a, b, c, d), reference_median(a, b, c, d));
}

TEST(DepthVideo, DepthToGreyMatchesReferenceWithStrides) {
  constexpr int w = 848, h = 480, z_stride = w + 16, luma_stride = w / 2 + 8;
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> val(1, 12000), hole(0, 3);
  std::vector<uint16_t> z(size_t(z_stride) * h, 0xBEEF);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) z[size_t(y) * z_stride + x] = hole(rng) == 0 ? 0 : uint16_t(val(rng));
  const GreyLut l = lut();
  std::vector<uint8_t> luma(size_t(luma_stride) * h / 2, 0xAA);
  bot_realsense::depth_to_grey(z.data(), z_stride * sizeof(uint16_t), w, h, l, luma.data(), luma_stride);
  for (int y = 0; y < h / 2; ++y) {
    for (int x = 0; x < w / 2; ++x) {
      const uint16_t* r0 = &z[size_t(2 * y) * z_stride + 2 * x];
      const uint16_t* r1 = r0 + z_stride;
      ASSERT_EQ(luma[size_t(y) * luma_stride + x], l[reference_median(r0[0], r0[1], r1[0], r1[1])]) << x << "," << y;
    }
    for (int x = w / 2; x < luma_stride; ++x) ASSERT_EQ(luma[size_t(y) * luma_stride + x], 0xAA);
  }
}

TEST(DepthVideo, LutEdges) {
  const GreyLut l = lut();
  EXPECT_EQ(l[0], 0);
  for (int v = 1; v <= 300; ++v) ASSERT_EQ(l[v], 16) << v;
  for (int v = 8000; v < 65536; ++v) ASSERT_EQ(l[v], 235) << v;
  for (int v = 1; v < 65536; ++v) ASSERT_TRUE(l[v] >= 16 && l[v] <= 235) << v;
  EXPECT_LT(l[301], 18);
  EXPECT_GT(l[7999], 233);
}

TEST(DepthVideo, LutIsMonotonic) {
  const GreyLut l = lut();
  for (int v = 2; v < 65536; ++v) ASSERT_GE(l[v], l[v - 1]) << v;
}

TEST(DepthVideo, InverseRoundTripsWithinOneCode) {
  const GreyLut l = lut();
  for (int y = 16; y <= 235; ++y) {
    const long v = std::lround(grey_to_metres(y) / kScale);
    ASSERT_LE(std::abs(int(l[v]) - y), 1) << y;
  }
  for (int v = 300; v <= 8000; ++v) {
    const double z = v * double(kScale), back = grey_to_metres(l[v]);
    const double y_exact = 16 + 219 * (1 / std::sqrt(double(kMin)) - 1 / std::sqrt(z)) /
                                    (1 / std::sqrt(double(kMin)) - 1 / std::sqrt(double(kMax)));
    const double y_back = 16 + 219 * (1 / std::sqrt(double(kMin)) - 1 / std::sqrt(back)) /
                                   (1 / std::sqrt(double(kMin)) - 1 / std::sqrt(double(kMax)));
    ASSERT_LE(std::abs(y_exact - y_back), 1.0) << v;
  }
}

TEST(DepthVideo, LutAppliesDepthScale) {
  const GreyLut mm = lut();
  GreyLut tenth_mm;  // D405-style 0.1 mm units
  bot_realsense::fill_grey_lut(tenth_mm, 0.0001f, kMin, kMax);
  for (int v = 1; v <= 6553; ++v) ASSERT_EQ(tenth_mm[10 * v], mm[v]) << v;
}
