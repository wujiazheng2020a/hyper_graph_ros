/**
 * @file network_bridge.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief achievement for network bridge
 * @version 0.1
 * @date 2023-08-15
 *
 *
 */

// #include <vector>
// #include <string>
// #include <yaml-cpp/yaml.h>

#include "tros_base/network_bridge/network_bridge.h"

namespace tros {
namespace tros_base {
NetworkBridge::NetworkBridge() : subscriber_running_(true) {}

NetworkBridge::~NetworkBridge() {
  for (auto& subscriber_socket : subscribers_) {
    subscriber_socket->close();
  }

  subscriber_running_.store(false);
  for (auto& sub_thread : sub_threads_) {
    if (sub_thread.joinable()) {
      sub_thread.join();
    }
  }
}

void NetworkBridge::Init(const std::string& network_config_path) {
  YAML::Node config = YAML::LoadFile(network_config_path);

  // 1. for publisher
  std::string publisher_ip =
      config["network"]["publisher"]["ip"].as<std::string>();
  if (publisher_ip != kNoIP) {
    LOG(INFO) << "<NetworkBridge> bind pub ip: " << publisher_ip << " success!";
    publisher_ =
        std::make_shared<zmqpp::socket>(context_, zmqpp::socket_type::publish);
    publisher_->bind(publisher_ip);

    // Subscribe to channels for publisher
    for (const auto& channel : config["network"]["publisher"]["channels"]) {
      std::string original_channel = channel.as<std::string>();
      std::string channel_name = original_channel + kNetworkOutChannelSuffix;
      pub_channels_.insert(original_channel);
      FromGraph<std::string>(
          channel_name, kWaitLastFinish,
          [this, original_channel](const std::string& data) {
            bool is_find;
            {
              std::shared_lock lock(publish_mutex_);  // read lock
              is_find =
                  (pub_channels_.find(original_channel) != pub_channels_.end());
            }
            if (is_find) {
              if (last_publish_channel_ != original_channel) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kChangePubChannelDelayMs));
              }
              this->publisher_->send(data);
              last_publish_channel_ = original_channel;
            }
          });
    }
  }

  // 2. for subscriber
  const YAML::Node& subscribers = config["network"]["subscribers"];
  for (const auto& subscriber : subscribers) {
    std::string subscriber_ip = subscriber["ip"].as<std::string>();
    if (subscriber_ip == kNoIP) {
      continue;
    }
    LOG(INFO) << "<NetworkBridge> bind sub ip: " << subscriber_ip
              << " success!";

    std::vector<std::string> channels =
        subscriber["channels"].as<std::vector<std::string>>();
    ZMQSocketPtr subscriber_socket = std::make_shared<zmqpp::socket>(
        context_, zmqpp::socket_type::subscribe);

    subscriber_socket->connect(subscriber_ip);
    subscriber_socket->set(zmqpp::socket_option::subscribe, "");
    subscribers_.emplace_back(subscriber_socket);

    std::set<std::string> now_channel_set;
    now_channel_set.insert(channels.begin(), channels.end());
    sub_channels.emplace_back(now_channel_set);

    sub_threads_.push_back(std::thread(&NetworkBridge::SubscribeZmqMessages,
                                       this, subscriber_socket,
                                       sub_channels.size() - 1));
  }
}

void NetworkBridge::ConfigPublishChannel(const std::string& channel_name,
                                         bool status) {
  std::unique_lock lock(publish_mutex_);  // read lock
  if (status) {
    pub_channels_.insert(channel_name);  // O(logn)
  } else {
    pub_channels_.erase(channel_name);  // O(logn)
  }
}

void NetworkBridge::ConfigSubscribeChannel(const std::string& channel_name,
                                           size_t subscriber_index,
                                           bool status) {
  std::unique_lock lock(sub_mutex_);  // read lock
  if (status) {
    sub_channels[subscriber_index].insert(channel_name);
  } else {
    sub_channels[subscriber_index].erase(channel_name);
  }
}

void NetworkBridge::SubscribeZmqMessages(ZMQSocketPtr subscriber,
                                         size_t index) {
  while (subscriber_running_.load()) {
    zmqpp::poller poller;
    poller.add(*subscriber);

    // Wait for 10ms or until there's something to read.
    // LOG(INFO) << "<NetworkBridge> polling!";
    if (poller.poll(10)) {
      VLOG(5) << "<NetworkBridge> start poll!";
      if (poller.has_input(*subscriber)) {
        VLOG(5) << "<NetworkBridge> receive data!";
        std::string received_data;
        subscriber->receive(received_data);

        auto channel_data_pair =
            NetworkEncapsulator::Decapsulate(received_data);

        bool is_find;
        {
          std::shared_lock lock(sub_mutex_);  // read lock
          is_find = (sub_channels[index].find(channel_data_pair.first) !=
                     sub_channels[index].end());
        }
        if (is_find) {
          channel_data_pair.first += kNetworkInChannelSuffix;

          // Send data to TBB graph
          VLOG(5) << "<NetworkBridge> receive zmq data and publish to TBB!";
          ToGraph(channel_data_pair.first, channel_data_pair.second);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void NetworkBridge::Stop() {
  for (auto& subscriber_socket : subscribers_) {
    subscriber_socket->close();
  }

  subscriber_running_.store(false);
}

}  // namespace tros_base
}  // namespace tros
