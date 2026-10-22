#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

namespace bot_realsense {

// When to try bringing a lost camera back: at once when armed, then once per period while it is absent, backing off
// (doubling up to max_ns) after each bring-up that failed with the camera present.
class RetrySchedule {
 public:
  RetrySchedule(int64_t period_ns, int64_t max_ns) : period_ns_(period_ns), max_ns_(max_ns) {}

  void arm(int64_t now_ns) {
    armed_ = true;
    due_ns_ = now_ns;
    backoff_ns_ = period_ns_;
  }
  void disarm() { armed_ = false; }
  bool armed() const { return armed_; }
  bool due(int64_t now_ns) const { return armed_ && now_ns >= due_ns_; }
  void camera_absent(int64_t now_ns) { due_ns_ = now_ns + period_ns_; }
  void bring_up_failed(int64_t now_ns) {
    due_ns_ = now_ns + backoff_ns_;
    backoff_ns_ = std::min(2 * backoff_ns_, max_ns_);
  }

 private:
  int64_t period_ns_, max_ns_;
  bool armed_ = false;
  int64_t due_ns_ = 0, backoff_ns_ = 0;
};

enum class DepthPick { kWait, kPrevious, kCurrent, kSkip };

// Which of two consecutive depth frames (stamps previous_ns < current_ns; previous_ns 0 if none) to register with a
// colour frame stamped colour_ns. Waits until a depth frame at or after the colour frame exists, then takes the
// nearer of the two; skips the colour frame if even that is more than max_gap_ns away.
inline DepthPick pick_depth_for(int64_t colour_ns, int64_t previous_ns, int64_t current_ns, int64_t max_gap_ns) {
  if (current_ns < colour_ns) return DepthPick::kWait;
  const int64_t to_current = current_ns - colour_ns;
  const int64_t to_previous = previous_ns != 0 ? (colour_ns > previous_ns ? colour_ns - previous_ns
                                                                          : previous_ns - colour_ns)
                                               : std::numeric_limits<int64_t>::max();
  const bool previous_nearer = to_previous < to_current;
  if ((previous_nearer ? to_previous : to_current) > max_gap_ns) return DepthPick::kSkip;
  return previous_nearer ? DepthPick::kPrevious : DepthPick::kCurrent;
}

}  // namespace bot_realsense
