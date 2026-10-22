#pragma once

#include <condition_variable>
#include <mutex>
#include <utility>

namespace bot_realsense {

// Drop-oldest handoff of one item from a producer callback to one consumer thread. T is a handle whose default
// value is empty and tests false (rs2::frame). Replaced and leftover items are released outside the lock.
template <class T>
class LatestSlot {
 public:
  // Replaces an unread item; dropped while closed.
  void put(T item) {
    T dropped;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return;
      dropped = std::exchange(item_, std::move(item));
    }
    ready_.notify_one();
  }

  // Blocks until an item arrives; an empty T once closed.
  T take() {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [this] { return closed_ || static_cast<bool>(item_); });
    return std::exchange(item_, T());
  }

  void open() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = false;
  }

  // Wakes the consumer with an empty T and drops any unread item.
  void close() {
    T dropped;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
      dropped = std::exchange(item_, T());
    }
    ready_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable ready_;
  T item_;
  bool closed_ = false;
};

}  // namespace bot_realsense
