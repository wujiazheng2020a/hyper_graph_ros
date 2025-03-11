/**
 * @file bridge_subscriber.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for local network test
 * @version 0.1
 * @date 2023-08-16
 *
 *
 */

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "tros_base/network_bridge/network_bridge.h"
#include "tros_base/transformation_tree/transformation_tree.h"

#include "tros_base/tros_base.h"

int main() {
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

  for (size_t i = 0; i < 100; ++i) {
    tros::tros_base::ToAny("tf_channel", tf_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LOG(ERROR) << "publish one data";
  }

  std::this_thread::sleep_for(std::chrono::seconds(50));
  net_bridge.Stop();

  return 0;
}
