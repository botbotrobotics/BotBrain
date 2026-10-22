#include <gtest/gtest.h>

#include "bot_realsense/timing.hpp"

using bot_realsense::DepthPick;
using bot_realsense::pick_depth_for;
using bot_realsense::RetrySchedule;

namespace {
constexpr int64_t kS = 1'000'000'000, kMs = 1'000'000;
}  // namespace

TEST(RetrySchedule, FirstTryAtOnceThenPollsWhileAbsent) {
  RetrySchedule r(2 * kS, 30 * kS);
  EXPECT_FALSE(r.due(0));
  r.arm(100 * kS);
  EXPECT_TRUE(r.due(100 * kS));
  r.camera_absent(100 * kS);
  EXPECT_FALSE(r.due(101 * kS));
  EXPECT_TRUE(r.due(102 * kS));
  r.camera_absent(102 * kS);
  EXPECT_TRUE(r.due(104 * kS));
}

TEST(RetrySchedule, FailedBringUpsBackOffToTheCapAndRearmResets) {
  RetrySchedule r(2 * kS, 30 * kS);
  r.arm(0);
  int64_t now = 0;
  for (const int64_t wait : {2, 4, 8, 16, 30, 30}) {
    r.bring_up_failed(now);
    EXPECT_FALSE(r.due(now + wait * kS - 1));
    EXPECT_TRUE(r.due(now + wait * kS));
    now += wait * kS;
  }
  r.arm(now);
  r.bring_up_failed(now);
  EXPECT_TRUE(r.due(now + 2 * kS));
  r.disarm();
  EXPECT_FALSE(r.due(now + 100 * kS));
}

TEST(PickDepthFor, WaitsForADepthFrameAtOrAfterTheColourFrame) {
  EXPECT_EQ(pick_depth_for(100 * kMs, 0, 90 * kMs, 70 * kMs), DepthPick::kWait);
  EXPECT_EQ(pick_depth_for(100 * kMs, 30 * kMs, 90 * kMs, 70 * kMs), DepthPick::kWait);
}

TEST(PickDepthFor, TakesTheNearerOfTheTwo) {
  EXPECT_EQ(pick_depth_for(100 * kMs, 90 * kMs, 156 * kMs, 70 * kMs), DepthPick::kPrevious);
  EXPECT_EQ(pick_depth_for(100 * kMs, 40 * kMs, 106 * kMs, 70 * kMs), DepthPick::kCurrent);
  EXPECT_EQ(pick_depth_for(100 * kMs, 0, 100 * kMs, 70 * kMs), DepthPick::kCurrent);
  EXPECT_EQ(pick_depth_for(100 * kMs, 120 * kMs, 186 * kMs, 70 * kMs), DepthPick::kPrevious);
  // A tie keeps the newer frame.
  EXPECT_EQ(pick_depth_for(100 * kMs, 80 * kMs, 120 * kMs, 70 * kMs), DepthPick::kCurrent);
}

TEST(PickDepthFor, SkipsWhenNoDepthFrameIsClose) {
  EXPECT_EQ(pick_depth_for(100 * kMs, 0, 500 * kMs, 70 * kMs), DepthPick::kSkip);
  EXPECT_EQ(pick_depth_for(100 * kMs, 10 * kMs, 300 * kMs, 70 * kMs), DepthPick::kSkip);
}
