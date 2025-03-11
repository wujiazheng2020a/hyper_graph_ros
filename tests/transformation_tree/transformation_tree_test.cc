/**
 * @file transformation_tree_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for tf tree test
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/transformation_tree/transformation_tree.h"

#include <gtest/gtest.h>

TEST(TransformationFrameTest, InitializationTest) {
  Eigen::Isometry3d tf;
  tf.setIdentity();  // 示例数据

  tros::tros_base::TransformationFrame frame(123456789, "parent", "now", tf);

  EXPECT_EQ(frame.time_stamp, 123456789);
  EXPECT_EQ(frame.parent_frame, "parent");
  EXPECT_EQ(frame.now_frame, "now");
  EXPECT_EQ(frame.tf_to_parent.matrix(), tf.matrix());
}

TEST(TransformationFrameTest, SerializationAndDeserializationTest) {
  Eigen::Isometry3d tf;
  tf.setIdentity();  // 示例数据

  tros::tros_base::TransformationFrame frame_original(123456789, "parent",
                                                      "now", tf);
  std::string serialized_data;
  EXPECT_TRUE(frame_original.SerializeToString(&serialized_data));

  tros::tros_base::TransformationFrame frame_deserialized;
  EXPECT_TRUE(frame_deserialized.ParseFromString(serialized_data));

  EXPECT_EQ(frame_original.time_stamp, frame_deserialized.time_stamp);
  EXPECT_EQ(frame_original.parent_frame, frame_deserialized.parent_frame);
  EXPECT_EQ(frame_original.now_frame, frame_deserialized.now_frame);
  EXPECT_EQ(frame_original.tf_to_parent.matrix(),
            frame_deserialized.tf_to_parent.matrix());
}

TEST(TransformationTreeTest, InsertSingleTF) {
  Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
  tf_translation.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  tros::tros_base::ToTF({100, "base_frame", "A", tf_translation});

  auto result = tros::tros_base::FromTF("base_frame", "A");
  EXPECT_EQ(result.time_stamp, 100);
  EXPECT_EQ(result.parent_frame, "base_frame");
  EXPECT_EQ(result.now_frame, "A");
  EXPECT_TRUE(result.tf_to_parent.isApprox(tf_translation));
}

TEST(TransformationTreeTest, InsertConsecutiveTF) {
  Eigen::Isometry3d tf_rotation = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd rotation_vector(
      M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 45 degree rotation around Z
  tf_rotation.rotate(rotation_vector);

  Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
  tf_translation.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  tros::tros_base::ToTF({100, "base_frame", "A", tf_translation});

  tros::tros_base::ToTF({200, "A", "B", tf_rotation});

  auto result = tros::tros_base::FromTF("base_frame", "B");
  EXPECT_EQ(result.time_stamp, 200);
  EXPECT_EQ(result.parent_frame, "base_frame");
  EXPECT_EQ(result.now_frame, "B");
  EXPECT_TRUE(result.tf_to_parent.isApprox(tf_translation * tf_rotation));
}

TEST(TransformationTreeTest, GetTransformRootToNode) {
  Eigen::Isometry3d tf_rotation = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd rotation_vector(
      M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 45 degree rotation around Z
  tf_rotation.rotate(rotation_vector);

  Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
  tf_translation.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  tros::tros_base::ToTF({100, "base_frame", "A", tf_translation});

  tros::tros_base::ToTF({300, "A", "B", tf_rotation});

  auto result = tros::tros_base::FromTF("A", "B");
  EXPECT_EQ(result.time_stamp, 300);
  EXPECT_TRUE(result.tf_to_parent.isApprox(tf_rotation));

  auto result2 = tros::tros_base::FromTF("B", "A");
  EXPECT_EQ(result2.time_stamp, 300);
  EXPECT_TRUE(result2.tf_to_parent.isApprox(tf_rotation.inverse()));
}

TEST(TransformationTreeTest, GetTransformBetweenAnyTwoNodes) {
  Eigen::Isometry3d tf_rotation = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd rotation_vector(
      M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 45 degree rotation around Z axis
  tf_rotation.rotate(rotation_vector);

  Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
  tf_translation.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

  tros::tros_base::ToTF({100, "base_frame", "A", tf_translation});

  tros::tros_base::ToTF({200, "A", "B", tf_rotation});

  tros::tros_base::ToTF({300, "base_frame", "C", tf_translation});

  tros::tros_base::ToTF({400, "C", "D", tf_rotation});

  tros::tros_base::ToTF({500, "C", "E", tf_translation});

  tros::tros_base::ToTF({600, "E", "F", tf_rotation});

  auto result = tros::tros_base::FromTF("base_frame", "D");
  EXPECT_EQ(result.time_stamp, 400);
  EXPECT_TRUE(result.tf_to_parent.isApprox(tf_translation * tf_rotation));

  auto result2 = tros::tros_base::FromTF("base_frame", "B");
  EXPECT_EQ(result2.time_stamp, 200);
  EXPECT_TRUE(result2.tf_to_parent.isApprox(result.tf_to_parent));

  auto result3 = tros::tros_base::FromTF("B", "D");
  EXPECT_EQ(result3.time_stamp, 400);
  EXPECT_TRUE(result3.tf_to_parent.isApprox(Eigen::Isometry3d::Identity()));

  auto result4 = tros::tros_base::FromTF("B", "F");
  EXPECT_EQ(result4.time_stamp, 600);
  EXPECT_TRUE(result4.tf_to_parent.isApprox(tf_rotation.inverse() *
                                            tf_translation * tf_rotation));

  auto result5 = tros::tros_base::FromTF("E", "A");
  EXPECT_EQ(result5.time_stamp, 500);
  EXPECT_TRUE(result5.tf_to_parent.isApprox(tf_translation.inverse()));

  auto result6 = tros::tros_base::FromTF("A", "E");
  EXPECT_EQ(result6.time_stamp, 500);
  EXPECT_TRUE(result6.tf_to_parent.isApprox(tf_translation));
}

TEST(TransformationTreeTest, GetTransformFromDiscontinuousPath) {
  auto result = tros::tros_base::FromTF("E", "Q");
  EXPECT_EQ(result.now_frame, "NoFind");

  auto result2 = tros::tros_base::FromTF("Q", "B");
  EXPECT_EQ(result2.now_frame, "NoFind");
}
