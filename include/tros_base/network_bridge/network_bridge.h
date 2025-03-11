/**
 * @file network_bridge.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief using zmq for multiple robot connect
 * @version 0.1
 * @date 2023-08-15
 *
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#include <zmqpp/zmqpp.hpp>
#include <shared_mutex>

#include "tros_base/data_graph/data_graph.h"
#include "tros_base/network_bridge/network_encapsulator.h"

namespace tros {
namespace tros_base {
typedef std::shared_ptr<zmqpp::socket> ZMQSocketPtr;

constexpr const char *kNoIP = "x";
constexpr const size_t kChangePubChannelDelayMs = 50;

class NetworkBridge {
 public:
  /**
   * @brief Construct a new Network Bridge object
   *
   */
  NetworkBridge();

  /**
   * @brief Destroy the Network Bridge object
   *
   */
  ~NetworkBridge();

  /**
   * @brief Init network bridge
   *
   * @param network_config_path for config
   */
  void Init(const std::string &network_config_path);

  /**
   * @brief stop network bridge
   *
   */
  void Stop();

  /**
   * @brief dynamic config channel
   *
   * @param channel_name
   * @param status open or close
   */
  void ConfigPublishChannel(const std::string &channel_name, bool status);

  /**
   * @brief dynamic config channel
   *
   * @param channel_name
   * @param subscriber_index
   * @param status
   */
  void ConfigSubscribeChannel(const std::string &channel_name,
                              size_t subscriber_index, bool status);

 public:
  static NetworkBridge &Instance() {
    static NetworkBridge instance;
    return instance;
  }

 private:
  /**
   * @brief subscribe one msg
   *
   * @param subscriber
   * @param index
   */
  void SubscribeZmqMessages(ZMQSocketPtr subscriber, size_t index);

 private:
  std::atomic<bool> subscriber_running_;
  std::vector<std::set<std::string>> sub_channels;
  std::set<std::string> pub_channels_;
  std::shared_mutex publish_mutex_;
  std::shared_mutex sub_mutex_;

  zmqpp::context context_;
  ZMQSocketPtr publisher_;
  std::string last_publish_channel_;
  std::vector<ZMQSocketPtr> subscribers_;
  std::vector<std::thread> sub_threads_;
};

typedef std::shared_ptr<NetworkBridge> NetworkBridgePtr;

/**
 * @brief Publish A data to graph or network
 *
 * @tparam DataType
 * @param channel_name channel_name of data,
 * @param data real data Should be shared_ptr
 */
template <typename DataType>
static void ToAny(const std::string &channel_name,
                  const std::shared_ptr<DataType> &data) {
  ToGraph(channel_name, data);

  std::string channel_with_suffix = channel_name + kNetworkOutChannelSuffix;
  if (BeSubscribed(channel_with_suffix)) {
    // capsule
    data->mutable_header()->set_channel_name(channel_name);
    std::string mix_data_str =
        NetworkEncapsulator::Encapsulate(channel_name, data);
    ToGraph(channel_with_suffix, mix_data_str);
  }
}

/**
 * @brief Subscribe data from graph or network, must declare first.
 *
 * @tparam DataType inout data type, Should be shared_ptr
 * @param channel_name channel name
 * @param call_back should be void(const DataType&)
 * @param is_concurrency_run if true, then callback function will run
 *                           concurrently,
 *                           if false, the callback function
 *                           will call only when last call finish
 */
template <typename DataType>
static void FromAny(
    const std::string &channel_name, const GraphRunModel &is_concurrency_run,
    std::function<void(const std::shared_ptr<DataType> &)> call_back) {
  DataGraph::Instance().Subscribe(channel_name, is_concurrency_run, call_back);

  std::string channel_with_suffix = channel_name + kNetworkInChannelSuffix;
  DataGraph::Instance().Subscribe<std::string>(
      channel_with_suffix, is_concurrency_run,
      [call_back](const std::string &data_str) {
        std::shared_ptr<DataType> shared_data = std::make_shared<DataType>();
        if (shared_data->ParseFromString(data_str)) {
          call_back(shared_data);
        }
      });
}

}  // namespace tros_base
}  // namespace tros
