/**
 * @file network_bridge_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief achievement for network bridge
 * @version 0.1
 * @date 2023-08-15
 *
 *
 */

#include "tros_base/network_bridge/network_bridge.h"

#include <gtest/gtest.h>

#include "tros_base/transformation_tree/transformation_tree.h"

TEST(NetworkBridgeTest, NetworkEncapsulatorTest) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  std::shared_ptr<tros::tros_base::TransformationFrame> original_frame =
      std::make_shared<tros::tros_base::TransformationFrame>(
          123456789, "parent_frame_example", "now_frame_example", isometry);

  // 使用NetworkEncapsulator进行封装
  std::string encapsulated_data =
      tros::tros_base::NetworkEncapsulator::Encapsulate("test_channel",
                                                        original_frame);

  // 使用NetworkEncapsulator进行解封装
  auto decapsulated_pair =
      tros::tros_base::NetworkEncapsulator::Decapsulate(encapsulated_data);

  // 验证通道名称是否正确
  EXPECT_EQ("test_channel", decapsulated_pair.first);

  // 反序列化解封装后的数据到新的TransformationFrame
  std::shared_ptr<tros::tros_base::TransformationFrame> deserialized_frame =
      std::make_shared<tros::tros_base::TransformationFrame>();
  ASSERT_TRUE(deserialized_frame->ParseFromString(decapsulated_pair.second));

  // 验证原始和反序列化后的TransformationFrame数据是否相同
  EXPECT_EQ(original_frame->time_stamp, deserialized_frame->time_stamp);
  EXPECT_EQ(original_frame->parent_frame, deserialized_frame->parent_frame);
  EXPECT_EQ(original_frame->now_frame, deserialized_frame->now_frame);
  EXPECT_EQ(original_frame->tf_to_parent.matrix(),
            deserialized_frame->tf_to_parent.matrix());
}

TEST(NetworkBridgeTest, LocalPublishTest) {
  tros::tros_base::NetworkBridge net_bridge;
  net_bridge.Init("../tests/data/network_test_sub.yaml");

  tros::tros_base::FromAny<tros::tros_base::TFFrame>(
      "tf_channel", tros::tros_base::kParallelInvoke,
      [](const std::shared_ptr<tros::tros_base::TFFrame>& tf_frame) {
        LOG(ERROR) << "data: " << tf_frame->parent_frame << ","
                   << tf_frame->tf_to_parent.matrix() << std::endl;
      });

  Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
  tf_translation.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  std::shared_ptr<tros::tros_base::TFFrame> tf_frame =
      std::make_shared<tros::tros_base::TFFrame>(100, "sub_frame", "A",
                                                 tf_translation);

  tros::tros_base::SetCloseChannel("tf_channel", true);

  for (size_t i = 0; i < 10; ++i) {
    tros::tros_base::ToAny("tf_channel", tf_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG(ERROR) << "publish one data";
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));
  net_bridge.Stop();
}
