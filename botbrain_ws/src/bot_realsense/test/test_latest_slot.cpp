#include <gtest/gtest.h>

#include <atomic>
#include <optional>
#include <thread>

#include "bot_realsense/latest_slot.hpp"

using Slot = bot_realsense::LatestSlot<std::optional<int>>;

TEST(LatestSlot, KeepsOnlyTheNewest) {
  Slot slot;
  slot.put(1);
  slot.put(2);
  EXPECT_EQ(slot.take(), 2);
}

TEST(LatestSlot, CloseWakesABlockedTakerAndDropsTheUnread) {
  Slot slot;
  std::optional<int> got = 0;
  std::thread taker([&] { got = slot.take(); });
  slot.close();
  taker.join();
  EXPECT_FALSE(got.has_value());

  slot.put(5);
  slot.open();
  slot.put(6);
  EXPECT_EQ(slot.take(), 6);
}

TEST(LatestSlot, ConsumerSeesIncreasingValuesAndStopsOnClose) {
  Slot slot;
  constexpr int kItems = 200000;
  std::atomic<int> taken{0};
  std::atomic<bool> ordered{true};
  std::thread consumer([&] {
    int last = 0;
    while (const std::optional<int> v = slot.take()) {
      if (*v <= last) ordered = false;
      last = *v;
      ++taken;
    }
  });
  for (int i = 1; i <= kItems; ++i) slot.put(i);
  slot.close();
  consumer.join();
  EXPECT_TRUE(ordered);
  EXPECT_GT(taken.load(), 0);
  EXPECT_LE(taken.load(), kItems);
}
