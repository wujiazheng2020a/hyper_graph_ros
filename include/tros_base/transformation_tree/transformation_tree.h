/**
 * @file transformation_tree.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief this class for transformation tree
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#pragma once

#include <glog/logging.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "tros_base/network_bridge/network_data.h"
#include "tros_base/proto/base_geometry.pb.h"

namespace tros {
namespace tros_base {

struct TransformationFrame : public NetworkData {
  uint64_t time_stamp;
  std::string parent_frame;
  std::string now_frame;
  Eigen::Isometry3d tf_to_parent;

  TransformationFrame() {}

  TransformationFrame(uint64_t time_stamp_in,
                      const std::string& parent_frame_in,
                      const std::string& now_frame_in,
                      const Eigen::Isometry3d& tf_to_parent_in)
      : time_stamp(time_stamp_in),
        parent_frame(parent_frame_in),
        now_frame(now_frame_in),
        tf_to_parent(tf_to_parent_in) {}

  ~TransformationFrame() {}

  bool SerializeToString(std::string* output) override {
    if (!output) {
      return false;
    }
    geometry::TransformationFrame tf_frame;
    tf_frame.set_time_stamp(time_stamp);
    tf_frame.set_parent_frame(parent_frame);
    tf_frame.set_now_frame(now_frame);

    // Convert Eigen::Isometry3d to repeated float
    Eigen::Matrix4d matrix = tf_to_parent.matrix();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        tf_frame.add_tf_to_parent(matrix(i, j));
      }
    }

    return tf_frame.SerializeToString(output);
  }

  bool ParseFromString(const std::string& data) override {
    geometry::TransformationFrame tf_frame;
    if (!tf_frame.ParseFromString(data)) {
      return false;
    }

    time_stamp = tf_frame.time_stamp();
    parent_frame = tf_frame.parent_frame();
    now_frame = tf_frame.now_frame();

    if (tf_frame.tf_to_parent_size() != 16) {
      return false;
    }

    Eigen::Matrix4d matrix;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        matrix(i, j) = tf_frame.tf_to_parent(i * 4 + j);
      }
    }
    tf_to_parent.matrix() = matrix;

    return true;
  }
};

using TFNode = TransformationFrame;
using TFFrame = TransformationFrame;

class TransformationTree {
 public:
  typedef std::unordered_map<std::string, TFNode> TFNodeMap;

 public:
  /**
   * @brief Construct a new Transformation Tree object
   *
   */
  TransformationTree();

  /**
   * @brief Destroy the Transformation Tree object
   *
   */
  ~TransformationTree();

  /**
   * @brief insert one tf_frame
   *
   * @param tf_frame one frame tf_frame
   */
  void Insert(const TFFrame& tf_frame);

  /**
   * @brief Get the Latest T F object
   *
   * @param parent_frame
   * @param now_frame
   * @return TFFrame if return frame equals to "NoFind", it means failure
   */
  TFFrame GetLatestTF(const std::string& parent_frame,
                      const std::string& now_frame);

 public:
  /**
   * @brief get a tf tree instance
   *
   * @return TransformationTree&
   */
  static TransformationTree& Instance() {
    static TransformationTree instance;
    return instance;
  }

 private:
  std::shared_mutex mutex_;
  std::unordered_map<std::string, TFNode> nodes_;
};

/**
 * @brief publish one tf_frame
 *
 * @param tf_frame one frame tf_frame
 */
static void ToTF(const TFFrame& tf_frame) {
  TransformationTree::Instance().Insert(tf_frame);
}

/**
 * @brief Get the Latest T F object
 *
 * @param parent_frame
 * @param now_frame
 * @return TFFrame if return frame equals to "NoFind", it means failure
 */
static TFFrame FromTF(const std::string& parent_frame,
                      const std::string& now_frame) {
  return TransformationTree::Instance().GetLatestTF(parent_frame, now_frame);
}

}  // namespace tros_base
}  // namespace tros
