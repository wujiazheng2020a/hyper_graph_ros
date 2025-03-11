/**
 * @file time_system.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief achieve time system
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/time_system/time_system.h"

namespace tros {
namespace tros_base {

TimeSystem::TimeSystem() : base_time_(0), real_time_in_base_time_(0) {}

TimeSystem::~TimeSystem() = default;

void TimeSystem::SetBaseTime(const uint64_t &base_time) {
  std::unique_lock get_time_lock(mutex_);
  real_time_in_base_time_ = GetRealTime();
  base_time_ = base_time;
}

uint64_t TimeSystem::GetRealTime() {
  auto current_time_chrono = std::chrono::system_clock::now();
  std::chrono::microseconds us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          current_time_chrono.time_since_epoch());

  return us.count();
}

uint64_t TimeSystem::GetRelativeTime() {
  std::shared_lock get_time_lock(mutex_);

  uint64_t real_time = GetRealTime();
  return base_time_ + real_time - real_time_in_base_time_;
}

}  // namespace tros_base
}  // namespace tros
