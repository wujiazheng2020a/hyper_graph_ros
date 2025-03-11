/**
 * @file time_system.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for time sub system, will be used in simulation model
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#pragma once

#include <chrono>
#include <iostream>
#include <shared_mutex>

namespace tros {
namespace tros_base {
typedef uint64_t TimeType;
class TimeSystem {
 public:
  /**
   * @brief Construct a new Time System object
   *
   */
  TimeSystem();

  /**
   * @brief Destroy the Time System object
   *
   */
  ~TimeSystem();

  /**
   * @brief Set the Base Time, unit: us,
   *        usally set as the initial time of dataset
   *        if not set, GetRelativeTime() will return real time
   *
   * @param base_time unit: us
   */
  void SetBaseTime(const uint64_t &base_time);

  /**
   * @brief Get the real time now, unit: us
   *
   * @return uint64_t real time(ms)
   */
  uint64_t GetRealTime();

  /**
   * @brief Get the relative time now,
   *        if not set, it will return real time
   * @details Namely: BaseTime + TimeFlow
   *
   * @return uint64_t relative time(us)
   */
  uint64_t GetRelativeTime();

 public:
  /**
   * @brief Get one instance of time system
   *
   * @return TimeSystem&
   */
  static TimeSystem &Instance() {
    static TimeSystem instance;
    return instance;
  }

 private:
  uint64_t base_time_;
  uint64_t real_time_in_base_time_;
  std::shared_mutex mutex_;
};

// for simplying
static TimeSystem& GetTimeSystem() {
  return TimeSystem::Instance();
}
}  // namespace tros_base
}  // namespace tros
