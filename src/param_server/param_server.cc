/**
 * @file param_server.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief
 * @version 0.1
 * @date 2023-08-13
 *
 *
 */

#include "tros_base/param_server/param_server.h"

namespace tros {
namespace tros_base {

ParamServer::ParamServer() : first_read_(false) {}

ParamServer::~ParamServer() = default;

const YAML::Node& ParamServer::Get() const {
  if (!first_read_) {
    LOG(ERROR) << "<ParamServer> not load param file first!";
  }
  return top_yaml_node_;
}

void ParamServer::ReadParam(const std::string& top_param_file_path) {
  top_yaml_node_ = YAML::LoadFile(top_param_file_path);
  first_read_ = true;
}

}  // namespace tros_base
}  // namespace tros
