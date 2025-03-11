/**
 * @file data_graph.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for data graph, used for parallel computing
 * @version 0.1
 * @date 2023-08-12
 *
 *
 */

#pragma once

#include <glog/logging.h>
#include <tbb/flow_graph.h>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <iostream>
#include <memory>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tros {
namespace tros_base {
typedef std::shared_ptr<void> AnyNode;
typedef std::unordered_map<std::string, AnyNode> ChannelNodeMap;
typedef std::unordered_map<std::string, bool> ChannelCloseMap;
typedef std::vector<AnyNode> FunctionNodeVector;

enum class GraphRunModel { wait_last_finish, parallel_invoke };
constexpr const char *kNetworkOutChannelSuffix = "##";
constexpr const char *kNetworkInChannelSuffix = "#";

/**
 * @brief data graph is a singleton for data propagation
 *
 */
class DataGraph {
 public:
  /**
   * @brief Construct a new Data Graph object
   *
   */
  DataGraph();

  /**
   * @brief Destroy the Data Graph object
   *
   */
  ~DataGraph();

  /**
   * @brief cancel and wait tbb graph thread join, must put in the last line
   *
   */
  void CancelAndWaitFinish();

  /**
   * @brief Publish A data to corresponding subscriber
   *
   * @tparam DataType
   * @param channel_name channel_name of data
   * @param data real data
   */
  template <typename DataType>
  void Publish(const std::string &channel_name, const DataType &data) {
    {
      std::shared_lock lock(close_map_mutex_);  // read lock
      auto it = channel_close_map_.find(channel_name);
      // if not find, permit publishing
      if (it != channel_close_map_.end() && it->second) {
        return;
      }
    }
    std::shared_lock lock(mutex_);  // read lock
    auto it = channel_node_map_.find(channel_name);
    if (it != channel_node_map_.end()) {
      auto broadcast_node =
          std::static_pointer_cast<tbb::flow::broadcast_node<DataType>>(
              it->second);
      broadcast_node->try_put(data);
    }
  }

  /**
   * @brief for dynamic close channel
   *
   * @param channel_name
   * @param status bool, true = close, false = not close
   */
  void SetCloseChannel(const std::string &channel_name, bool status);

  /**
   * @brief For Init Setting
   *
   * @param graph_config_path
   */
  void InitCloseChannel(const std::string &graph_config_path);

  /**
   * @brief check if this channel is subscribed
   *
   * @param channel_name
   * @return true
   * @return false
   */
  bool BeSubscribed(const std::string &channel_name);

  /**
   * @brief Subscribe data from given channel, must declare first.
   *
   * @tparam DataType inout data type
   * @param channel_name channel name
   * @param call_back should be void(const DataType&)
   * @param is_concurrency_run if true, then callback function will run
   *                           concurrently,
   *                           if false, the callback function
   *                           will call only when last call finish
   */
  template <typename DataType>
  void Subscribe(const std::string &channel_name,
                 const GraphRunModel &is_concurrency_run,
                 std::function<void(const DataType &)> call_back) {
    tbb::detail::d1::concurrency is_concurrency = tbb::flow::unlimited;
    if (is_concurrency_run == GraphRunModel::wait_last_finish) {
      is_concurrency = tbb::flow::serial;
    }
    std::shared_ptr<tbb::flow::function_node<DataType, tbb::flow::continue_msg>>
        my_func_node = std::make_shared<
            tbb::flow::function_node<DataType, tbb::flow::continue_msg>>(
            graph_, is_concurrency,
            [call_back](const DataType &data) -> tbb::flow::continue_msg {
              call_back(data);
              return tbb::flow::continue_msg();
            });

    std::unique_lock lock(mutex_);  // write lock
    std::shared_ptr<tbb::flow::broadcast_node<DataType>> my_broadcast_node;
    auto it = channel_node_map_.find(channel_name);
    if (it != channel_node_map_.end()) {
      my_broadcast_node =
          std::static_pointer_cast<tbb::flow::broadcast_node<DataType>>(
              it->second);
    } else {
      my_broadcast_node =
          std::make_shared<tbb::flow::broadcast_node<DataType>>(graph_);
      channel_node_map_.insert(std::make_pair(
          channel_name, std::static_pointer_cast<void>(my_broadcast_node)));
    }

    tbb::flow::make_edge(*my_broadcast_node, *my_func_node);
    function_node_vector_.emplace_back(
        std::static_pointer_cast<void>(my_func_node));
  }

 public:
  /**
   * @brief Get One Instance of data_graph
   *
   * @return DataGraph&
   */
  static DataGraph &Instance() {
    static DataGraph instance;
    return instance;
  }

 private:
  std::shared_mutex mutex_;
  std::shared_mutex close_map_mutex_;
  tbb::flow::graph graph_;
  ChannelNodeMap channel_node_map_;
  ChannelCloseMap channel_close_map_;
  FunctionNodeVector function_node_vector_;
};

// for simplifying
constexpr const GraphRunModel kWaitLastFinish = GraphRunModel::wait_last_finish;
constexpr const GraphRunModel kParallelInvoke = GraphRunModel::parallel_invoke;

/**
 * @brief Publish A data to corresponding subscriber
 *
 * @tparam DataType
 * @param channel_name channel_name of data
 * @param data real data
 */
template <typename DataType>
static void ToGraph(const std::string &channel_name, const DataType &data) {
  DataGraph::Instance().Publish(channel_name, data);
}

/**
 * @brief Subscribe data from given channel, must declare first.
 *
 * @tparam DataType inout data type
 * @param channel_name channel name
 * @param call_back should be void(const DataType&)
 * @param is_concurrency_run if true, then callback function will run
 *                           concurrently,
 *                           if false, the callback function
 *                           will call only when last call finish
 */
template <typename DataType>
static void FromGraph(const std::string &channel_name,
                      const GraphRunModel &is_concurrency_run,
                      std::function<void(const DataType &)> call_back) {
  DataGraph::Instance().Subscribe(channel_name, is_concurrency_run, call_back);
}

/**
 * @brief check if this channel is subscribed
 *
 * @param channel_name
 * @return true
 * @return false
 */
static bool BeSubscribed(const std::string &channel_name) {
  return DataGraph::Instance().BeSubscribed(channel_name);
}

/**
 * @brief for dynamic close channel
 *
 * @param channel_name
 * @param status bool, true = close, false = not close
 */
static void SetCloseChannel(const std::string &channel_name, bool status) {
  DataGraph::Instance().SetCloseChannel(channel_name, status);
}

static void InitCloseChannel(const std::string &graph_config_path) {
  DataGraph::Instance().InitCloseChannel(graph_config_path);
}

}  // namespace tros_base
}  // namespace tros
