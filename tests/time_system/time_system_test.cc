/**
 * @file time_system_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for time system test
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/time_system/time_system.h"

#include <gtest/gtest.h>

#include <thread>

TEST(WJZ_SYSTEM, TIME_SYSTEM) {
  tros::tros_base::GetTimeSystem().SetBaseTime(1403636579758555);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_NEAR(tros::tros_base::GetTimeSystem().GetRelativeTime(),
              1403636579758555 + 100000, 1000);
}
