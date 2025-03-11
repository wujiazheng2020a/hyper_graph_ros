/**
 * @file tros_base.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief facade of tros
 * @version 0.1
 * @date 2023-08-17
 *
 *
 */

#pragma once

#include <glog/logging.h>

#include <string>

#include "tros_base/proto/navigation_message.pb.h"
#include "tros_base/data_graph/data_graph.h"
#include "tros_base/network_bridge/network_bridge.h"
#include "tros_base/param_server/param_server.h"
#include "tros_base/time_system/time_system.h"
#include "tros_base/transformation_tree/transformation_tree.h"
#include "tros_base/proto/nav_geometry.pb.h"

namespace tros {
namespace tros_base {
static void TROSInit(const std::string& param_file_path,
                     const std::string& graph_file_path,
                     const std::string& network_config_path,
                     uint64_t base_time) {
  if (!param_file_path.empty()) {
    tros::tros_base::ParamsInit(param_file_path);
  } else {
    LOG(INFO) << "<TROS> not param file!";
  }

  if (!graph_file_path.empty()) {
    tros::tros_base::InitCloseChannel(graph_file_path);
  } else {
    LOG(INFO) << "<TROS> not graph file, all channel open!";
  }

  if (!network_config_path.empty()) {
    tros::tros_base::NetworkBridge::Instance().Init(network_config_path);
  } else {
    LOG(INFO) << "<TROS> not network config file, close network transmisson!";
  }
  tros::tros_base::GetTimeSystem().SetBaseTime(base_time);
}
}  // namespace tros_base
}  // namespace tros
