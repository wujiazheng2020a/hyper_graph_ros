/**
 * @file network_encapsulator.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief encapsulate the network data
 * @version 0.1
 * @date 2023-08-16
 *
 *
 */

#pragma once

#include <glog/logging.h>

#include <string>
#include <memory>

#include "tros_base/proto/common.pb.h"

namespace tros {
namespace tros_base {

constexpr const char* kHeaderSeparator = "$$";

struct NetworkEncapsulator {
  /**
   * @brief Encapsulate the data for network transmission
   *
   * @tparam DataType
   * @param channel_name
   * @param data must be shared_ptr
   * @return std::string
   */
  template <typename DataType>
  static std::string Encapsulate(const std::string& channel_name,
                                 const std::shared_ptr<DataType>& data) {
    std::string header_str, data_str;

    // Set the channel name in the header
    data->mutable_header()->set_channel_name(channel_name);

    // Serialize the header and data
    data->mutable_header()->SerializeToString(&header_str);
    data->SerializeToString(&data_str);

    // Compute the length of the header string
    size_t header_length = header_str.size();

    // Create the final string
    std::string encapsulated_data = std::to_string(header_length) +
                                    kHeaderSeparator + header_str + data_str;

    return encapsulated_data;
  }

  // Decapsulate the received data, returning the channel name and the raw data
  // string
  static std::pair<std::string, std::string> Decapsulate(
      const std::string& received_data) {
    // Find the position of "$$"
    size_t pos = received_data.find(kHeaderSeparator);
    if (pos == std::string::npos) {
      LOG(FATAL) << "<NetworkEncapsulator> receive data error!";
    }

    // Extract header length
    size_t header_length = std::stoul(received_data.substr(0, pos));

    // Extract and parse the header string
    std::string header_str = received_data.substr(pos + 2, header_length);
    // Assume `HeaderType` is the protobuf message type for the header
    common::Header header;
    if (!header.ParseFromString(header_str)) {
      LOG(FATAL) << "<NetworkEncapsulator> receive data error!";
    }

    // Extract the channel name from the header
    std::string channel_name = header.channel_name();

    // Extract the original data string
    std::string data_str = received_data.substr(pos + 2 + header_length);

    return {channel_name, data_str};
  }
};

}  // namespace tros_base
}  // namespace tros
