/**
 * @file data_graph.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for data graph, used for parallel computing
 * @version 0.1
 * @date 2023-08-12
 *
 *
 */

#include "tros_base/data_graph/data_graph.h"

namespace tros {
namespace tros_base {

DataGraph::DataGraph() = default;

DataGraph::~DataGraph() = default;

void DataGraph::CancelAndWaitFinish() {
  graph_.cancel();
  graph_.wait_for_all();
  channel_node_map_.clear();
  function_node_vector_.clear();
  channel_close_map_.clear();
}

void DataGraph::SetCloseChannel(const std::string &channel_name, bool status) {
  std::unique_lock lock(close_map_mutex_);  // write lock
  channel_close_map_[channel_name] = status;
}

void DataGraph::InitCloseChannel(const std::string &graph_config_path) {
  YAML::Node config = YAML::LoadFile(graph_config_path);

  // 获取close_channels列表
  const YAML::Node &close_channels = config["data_graph"]["close_channels"];

  // 读取并打印所有的channels
  for (const auto &channel : close_channels) {
    SetCloseChannel(channel.as<std::string>(), true);
  }
}

bool DataGraph::BeSubscribed(const std::string &channel_name) {
  std::shared_lock lock(mutex_);  // read lock
  auto it = channel_node_map_.find(channel_name);
  if (it != channel_node_map_.end()) {
    return true;
  }
  return false;
}

}  // namespace tros_base
}  // namespace tros
