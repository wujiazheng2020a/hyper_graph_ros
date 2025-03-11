/**
 * @file network_structure.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief network_structure base class
 * @version 0.1
 * @date 2023-08-16
 *
 *
 */

#pragma once

#include "tros_base/proto/common.pb.h"

namespace tros {
namespace tros_base {
struct NetworkData {
  /**
   * @brief basic header, for multi robot use
   *
   */
  common::Header header;

  /**
   * @brief for consistency
   *
   * @return common::Header
   */
  common::Header* mutable_header() { return &header; }

  /**
   * @brief follow google protobuf style
   *
   * @param output Serialized Data
   * @return true
   * @return false
   */
  virtual bool SerializeToString(std::string* output) = 0;

  /**
   * @brief follow google protobuf style
   *
   * @param data struct data
   * @return true
   * @return false
   */
  virtual bool ParseFromString(const std::string& data) = 0;
};

}  // namespace tros_base
}  // namespace tros
