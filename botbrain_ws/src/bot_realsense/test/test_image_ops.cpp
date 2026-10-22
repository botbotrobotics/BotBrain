#include <gtest/gtest.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "bot_realsense/image_ops.hpp"

using bot_realsense::DepthAligner;
using bot_realsense::RateLimiter;

namespace {

constexpr int64_t kMs = 1000000;

// Frames at `fps` with a deterministic +-2 ms jitter.
std::vector<int64_t> run_limiter(double fps, double hz, double seconds, int64_t start_ns = 1'700'000'000'000 * kMs) {
  RateLimiter limiter(hz);
  std::vector<int64_t> passed;
  const int frames = static_cast<int>(fps * seconds);
  for (int i = 0; i < frames; ++i) {
    const int64_t jitter = ((i * 7919) % 5 - 2) * kMs;
    const int64_t t = start_ns + static_cast<int64_t>(i * 1e9 / fps) + jitter;
    if (limiter.allow(t)) passed.push_back(t);
  }
  return passed;
}

}  // namespace

TEST(RateLimiter, ZeroPassesEveryFrame) { EXPECT_EQ(run_limiter(30, 0, 10).size(), 300u); }

TEST(RateLimiter, AboveSourceRatePassesEveryFrame) { EXPECT_EQ(run_limiter(30, 60, 10).size(), 300u); }

TEST(RateLimiter, HalfRateTakesEveryOtherFrame) {
  const auto passed = run_limiter(30, 15, 10);
  EXPECT_NEAR(double(passed.size()), 150, 1);
  for (size_t i = 1; i < passed.size(); ++i) EXPECT_NEAR(double(passed[i] - passed[i - 1]) / kMs, 66.7, 5);
}

TEST(RateLimiter, SixHzFromThirtyIsEvenAndExact) {
  const auto passed = run_limiter(30, 6, 10);
  EXPECT_NEAR(double(passed.size()), 60, 1);
  for (size_t i = 1; i < passed.size(); ++i) {
    const double gap_ms = double(passed[i] - passed[i - 1]) / kMs;
    EXPECT_GT(gap_ms, 150);
    EXPECT_LT(gap_ms, 185);
  }
}

TEST(RateLimiter, NonDivisorRateKeepsItsAverage) { EXPECT_NEAR(double(run_limiter(30, 7, 20).size()), 140, 1); }

TEST(RateLimiter, RecoversAfterGapAndClockJumpBack) {
  RateLimiter limiter(6);
  const int64_t t0 = 1'000'000 * kMs;
  EXPECT_TRUE(limiter.allow(t0));
  EXPECT_FALSE(limiter.allow(t0 + 33 * kMs));
  EXPECT_TRUE(limiter.allow(t0 + 5000 * kMs));
  EXPECT_TRUE(limiter.allow(t0 - 3600'000 * kMs));
  EXPECT_FALSE(limiter.allow(t0 - 3600'000 * kMs + 33 * kMs));
}

TEST(YuyvToRgb8, PrimariesAndStride) {
  // 6x2 image, 16-byte rows (4 bytes padding). Row 0: white, black | red, red | blue, blue. Row 1: green x6.
  const std::array<uint8_t, 32> yuyv = {235, 128, 16, 128, 81, 90, 81, 240, 41, 240, 41, 110, 0, 0, 0, 0,
                                        145, 54, 145, 34, 145, 54, 145, 34, 145, 54, 145, 34, 0, 0, 0, 0};
  std::array<uint8_t, 6 * 2 * 3> rgb{};
  bot_realsense::yuyv_to_rgb8(yuyv.data(), 16, 6, 2, rgb.data());
  const std::array<std::array<int, 3>, 6> row0 = {{{255, 255, 255}, {0, 0, 0}, {255, 0, 0}, {255, 0, 0}, {0, 0, 255},
                                                   {0, 0, 255}}};
  for (int x = 0; x < 6; ++x) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(rgb[x * 3 + c], row0[x][c], 2) << "row 0 pixel " << x << " channel " << c;
      EXPECT_NEAR(rgb[18 + x * 3 + c], (c == 1) * 255, 2) << "row 1 pixel " << x << " channel " << c;
    }
  }
}

TEST(DepthToMm, CopiesMillimetresAndRescalesOtherUnits) {
  // 3x2 image with a 2-sample row pad.
  const std::array<uint16_t, 10> z = {0, 1000, 65534, 7, 7, 12346, 1, 10000, 9, 9};
  std::array<uint16_t, 6> mm{};
  bot_realsense::depth_to_mm(z.data(), 10, 3, 2, 0.001f, mm.data());
  EXPECT_EQ(mm, (std::array<uint16_t, 6>{0, 1000, 65534, 12346, 1, 10000}));
  bot_realsense::depth_to_mm(z.data(), 10, 3, 2, 0.0001f, mm.data());
  EXPECT_EQ(mm, (std::array<uint16_t, 6>{0, 100, 6553, 1235, 0, 1000}));
  bot_realsense::depth_to_mm(z.data(), 10, 3, 2, 0.01f, mm.data());
  EXPECT_EQ(mm, (std::array<uint16_t, 6>{0, 10000, 65535, 65535, 10, 65535}));
}

namespace {

std::vector<uint16_t> librealsense_align(const std::vector<uint16_t>& depth, const rs2_intrinsics& di,
                                         const rs2_extrinsics& depth_to_color, const rs2_intrinsics& ci, float scale) {
  rs2::software_device device;
  auto depth_sensor = device.add_sensor("Depth");
  auto color_sensor = device.add_sensor("Color");
  auto depth_profile =
      depth_sensor.add_video_stream({RS2_STREAM_DEPTH, 0, 0, di.width, di.height, 30, 2, RS2_FORMAT_Z16, di});
  auto color_profile =
      color_sensor.add_video_stream({RS2_STREAM_COLOR, 0, 1, ci.width, ci.height, 30, 3, RS2_FORMAT_RGB8, ci});
  depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, scale);
  depth_profile.register_extrinsics_to(color_profile, depth_to_color);

  rs2::frame_queue depth_queue(1, true), color_queue(1, true), set_queue(1, true);
  depth_sensor.open(depth_profile);
  color_sensor.open(color_profile);
  depth_sensor.start(depth_queue);
  color_sensor.start(color_queue);
  std::vector<uint8_t> rgb(size_t(ci.width) * ci.height * 3, 0);
  depth_sensor.on_video_frame({const_cast<uint16_t*>(depth.data()), [](void*) {}, di.width * 2, 2, 0.0,
                               RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1, depth_profile.get(), scale});
  color_sensor.on_video_frame({rgb.data(), [](void*) {}, ci.width * 3, 3, 0.0, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1,
                               color_profile.get(), 0.f});
  const rs2::frame depth_frame = depth_queue.wait_for_frame();
  const rs2::frame color_frame = color_queue.wait_for_frame();

  rs2::processing_block compose([&](rs2::frame, rs2::frame_source& source) {
    source.frame_ready(source.allocate_composite_frame({depth_frame, color_frame}));
  });
  compose.start(set_queue);
  compose.invoke(depth_frame);
  rs2::align align(RS2_STREAM_COLOR);
  const rs2::depth_frame aligned = align.process(rs2::frameset(set_queue.wait_for_frame())).get_depth_frame();
  const auto* data = static_cast<const uint16_t*>(aligned.get_data());
  std::vector<uint16_t> out(data, data + size_t(ci.width) * ci.height);
  depth_sensor.stop();
  color_sensor.stop();
  return out;
}

// Share of the colour pixels either side fills where ours and rs2::align agree within 1 mm, on a slanted wall at
// 1.2..4.2 m with a 0.6 m box (it occludes the wall from the colour view), noise and holes. Depth is D435i-like:
// undistorted 848x480, 15 mm baseline, a small rotation.
double agreement_with_librealsense(const rs2_intrinsics& ci) {
  const rs2_intrinsics di{848, 480, 424.3f, 239.6f, 421.5f, 421.5f, RS2_DISTORTION_BROWN_CONRADY, {0, 0, 0, 0, 0}};
  const float a = 0.01f;  // rad about y
  const rs2_extrinsics d2c{{std::cos(a), 0, -std::sin(a), 0, 1, 0, std::sin(a), 0, std::cos(a)},
                           {0.0149f, 0.0002f, 0.0004f}};
  std::vector<uint16_t> depth(size_t(di.width) * di.height);
  std::srand(7);
  for (int y = 0; y < di.height; ++y) {
    for (int x = 0; x < di.width; ++x) {
      uint16_t z = static_cast<uint16_t>(1200 + 3 * x + y + std::rand() % 7);
      if (x > 300 && x < 420 && y > 150 && y < 330) z = static_cast<uint16_t>(600 + std::rand() % 3);
      if ((x / 16 + y / 16) % 9 == 0 || std::rand() % 50 == 0) z = 0;
      depth[size_t(y) * di.width + x] = z;
    }
  }

  const std::vector<uint16_t> expected = librealsense_align(depth, di, d2c, ci, 0.001f);
  std::vector<uint16_t> ours(expected.size());
  DepthAligner(di, d2c, ci).align(depth.data(), di.width * 2, 0.001f, ours.data());

  size_t covered = 0, close = 0;
  for (size_t i = 0; i < ours.size(); ++i) {
    if (!expected[i] && !ours[i]) continue;
    ++covered;
    if (expected[i] && ours[i] && std::abs(int(expected[i]) - int(ours[i])) <= 1) ++close;
  }
  EXPECT_GT(covered, ours.size() / 2);
  return static_cast<double>(close) / static_cast<double>(covered);
}

}  // namespace

TEST(DepthAligner, MatchesLibrealsenseAlignOnD435iColour) {
  // The D435i reports inverse Brown-Conrady colour with zero coefficients.
  const rs2_intrinsics ci{640, 480, 302.5f, 250.7f, 606.6f, 606.3f, RS2_DISTORTION_INVERSE_BROWN_CONRADY,
                          {0, 0, 0, 0, 0}};
  EXPECT_GT(agreement_with_librealsense(ci), 0.999);
}

TEST(DepthAligner, MatchesLibrealsenseAlignWithLensDistortion) {
  // Modified Brown-Conrady: the model every rs2::align implementation (generic, SSE, NEON) distorts with.
  const rs2_intrinsics ci{640, 480, 318.2f, 244.9f, 607.1f, 606.8f, RS2_DISTORTION_MODIFIED_BROWN_CONRADY,
                          {0.11f, -0.23f, 0.0012f, -0.0007f, 0.09f}};
  EXPECT_GT(agreement_with_librealsense(ci), 0.999);
}
