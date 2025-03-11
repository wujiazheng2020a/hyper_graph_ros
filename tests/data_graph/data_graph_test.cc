/**
 * @file data_graph_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for data graph test
 * @version 0.1
 * @date 2023-08-12
 *
 *
 */

#include "tros_base/data_graph/data_graph.h"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

TEST(TROS, DATA_GRAPH_M2M_TEST) {
  tros::tros_base::InitCloseChannel("../tests/data/data_graph_setting.yaml");

  tros::tros_base::FromGraph<int>(
      "wheel_odom_channel", tros::tros_base::kWaitLastFinish,
      [](const int &data) {
        std::cout << "data1:" << data << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      });

  tros::tros_base::FromGraph<int>(
      "wheel_odom_channel", tros::tros_base::kParallelInvoke,
      [](const int &data) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        std::cout << "data2:" << data + 20 << std::endl;
      });

  tros::tros_base::FromGraph<int>(
      "imu_channel", tros::tros_base::kWaitLastFinish, [](const int &data) {
        EXPECT_EQ(data, 10);
        std::cout << "imu_channel:" << data << std::endl;
      });

  tros::tros_base::FromGraph<int>(
      "imu_channel", tros::tros_base::kWaitLastFinish, [](const int &data) {
        EXPECT_EQ(data, 10);
        std::cout << "imu_channel2:" << data << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      });

  tros::tros_base::FromGraph<std::shared_ptr<int>>(
      "shared_channel", tros::tros_base::kWaitLastFinish,
      [](const std::shared_ptr<int> &data) {
        EXPECT_EQ(*data, 88);
        std::cout << "shared_channel:" << *data << std::endl;
      });

  for (size_t i = 0; i < 10; ++i) {
    tros::tros_base::ToGraph("wheel_odom_channel", 5);
    tros::tros_base::ToGraph("wheel_odom_channel", 8);
    tros::tros_base::ToGraph("imu_channel", 10);
    tros::tros_base::ToGraph("shared_channel", std::make_shared<int>(88));
    if (i > 5) {
      tros::tros_base::SetCloseChannel("shared_channel", true);
    }
  }

  std::cout << "publish over!" << std::endl;

  EXPECT_EQ(tros::tros_base::BeSubscribed("imu_channel"), true);
  EXPECT_EQ(tros::tros_base::BeSubscribed("wheel_odom_channel"), true);
  EXPECT_EQ(tros::tros_base::BeSubscribed("test_channel"), false);

  std::this_thread::sleep_for(std::chrono::seconds(100));
  tros::tros_base::DataGraph::Instance().CancelAndWaitFinish();
}
