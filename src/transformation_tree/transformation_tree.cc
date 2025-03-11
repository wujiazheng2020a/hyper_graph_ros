/**
 * @file transformation_tree.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for tf tree achievement
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/transformation_tree/transformation_tree.h"

namespace tros {
namespace tros_base {

TransformationTree::TransformationTree() = default;

TransformationTree::~TransformationTree() = default;

void TransformationTree::Insert(const TFFrame& tf_frame) {
  std::unique_lock lock(mutex_);  // write lock
  // initial insert
  if (nodes_.empty()) {
    nodes_[tf_frame.parent_frame] = {tf_frame.time_stamp, "",
                                     tf_frame.parent_frame,
                                     Eigen::Isometry3d::Identity()};
  }
  nodes_[tf_frame.now_frame] = tf_frame;
}

TFFrame TransformationTree::GetLatestTF(const std::string& parent_frame,
                                        const std::string& now_frame) {
  std::shared_lock lock(mutex_);  // read lock

  // 1. start from child_frame
  std::list<TFNode> child_path;
  auto it = nodes_.find(now_frame);
  bool direct_find = false;
  while (it != nodes_.end()) {
    if (it->second.now_frame == parent_frame) {
      direct_find = true;
      break;
    }
    child_path.push_front(it->second);

    it = nodes_.find(it->second.parent_frame);
  }
  if (child_path.empty()) {
    LOG(ERROR) << "<TransformationTree> query no find!";
    return {0, "NoFind", "NoFind", Eigen::Isometry3d::Identity()};
  }

  Eigen::Isometry3d tf_final = Eigen::Isometry3d::Identity();

  uint64_t newest_time = 0;
  for (const auto& tf_inc : child_path) {
    tf_final = tf_final * tf_inc.tf_to_parent;
    if (tf_inc.time_stamp > newest_time) {
      newest_time = tf_inc.time_stamp;
    }
  }
  TFFrame tf_frame_child{newest_time, parent_frame, now_frame, tf_final};

  if (direct_find) {
    return tf_frame_child;
  }

  // 2. if not in child path, find in parent path
  std::list<TFNode> parent_path;
  it = nodes_.find(parent_frame);
  while (it != nodes_.end()) {
    if (it->second.now_frame == now_frame) {
      direct_find = true;
      break;
    }
    parent_path.push_front(it->second);

    it = nodes_.find(it->second.parent_frame);
  }

  if (parent_path.empty()) {
    LOG(ERROR) << "<TransformationTree> query no find!";
    return {0, "NoFind", "NoFind", Eigen::Isometry3d::Identity()};
  }

  tf_final = Eigen::Isometry3d::Identity();

  newest_time = 0;
  for (const auto& tf_inc : parent_path) {
    tf_final = tf_final * tf_inc.tf_to_parent;
    if (tf_inc.time_stamp > newest_time) {
      newest_time = tf_inc.time_stamp;
    }
  }

  TFFrame tf_frame_parent{newest_time, parent_frame, now_frame,
                          tf_final.inverse()};

  if (direct_find) {
    return tf_frame_parent;
  }

  // if not find, combine
  newest_time = tf_frame_parent.time_stamp > tf_frame_child.time_stamp
                    ? tf_frame_parent.time_stamp
                    : tf_frame_child.time_stamp;
  return {newest_time, parent_frame, now_frame,
          tf_frame_parent.tf_to_parent * tf_frame_child.tf_to_parent};
}

}  // namespace tros_base
}  // namespace tros
