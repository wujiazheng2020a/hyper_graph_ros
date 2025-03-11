/**
 * @file param_server_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for param server test
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/param_server/param_server.h"

#include <gtest/gtest.h>

#include <string>

TEST(TROS, PARAM_SERVER_MAIN) {
  EXPECT_EQ(tros::tros_base::Params().IsNull(), true);

  tros::tros_base::ParamsInit("../tests/data/param_file.yaml");

  const auto &my_name =
      tros::tros_base::Params()["profile"]["name"].as<std::string>();
  EXPECT_EQ(my_name, "wujiazheng");

  const auto &my_sex =
      tros::tros_base::Params()["profile"]["sex"].as<std::string>();
  EXPECT_EQ(my_sex, "male");

  const auto &car1 =
      tros::tros_base::Params()["profile"]["car_id"]["level1"].as<int>();
  EXPECT_EQ(car1, 122);

  const auto &car2 =
      tros::tros_base::Params()["profile"]["car_id"]["level2"].as<int>();
  EXPECT_EQ(car2, 233);

  const auto &imu = tros::tros_base::Params()["sensor"].as<std::string>();
  EXPECT_EQ(imu, "imu");
}
