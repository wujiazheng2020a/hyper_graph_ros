/**
 * @file param_server.h
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief this is a singleton for param_server
 *        based on yaml-cpp
 * @version 0.1
 * @date 2023-08-12
 *
 *
 */

#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <memory>

namespace tros {
namespace tros_base {
class ParamServer {
 public:
  /**
   * @brief Construct a new Param Server object
   *
   */
  ParamServer();

  /**
   * @brief Destroy the Param Server object
   *
   */
  ~ParamServer();

 public:
  /**
   * @brief get the singleton of paramserver
   *
   * @return ParamServer& param server singleton
   */
  static ParamServer& Instance() {
    static ParamServer instance;
    return instance;
  }

  /**
   * @brief get YAML NODE
   *        exp:  const auto& car2 =
   *                       param_server::ParamServer::Instance()
   *                      .Get()["profile"]["car_id"]["level2"]
   *                      .as<int>();
   *
   * @return YAML::Node real yamlnode
   */
  const YAML::Node& Get() const;

  /**
   * @brief init param server, must call when init
   *
   * @param top_param_file_path
   */
  void ReadParam(const std::string& top_param_file_path);

 private:
  bool first_read_;
  YAML::Node top_yaml_node_;
};

// for simplifying

/**
 * @brief init param server, must call when init
 *
 * @param param_file_path
 */
static void ParamsInit(const std::string& param_file_path) {
  ParamServer::Instance().ReadParam(param_file_path);
}

/**
 * @brief get param saved in param file
 *        exp:  const auto& car2 =
 *                       param_server::Params()["profile"]["car_id"]["level2"]
 *                      .as<int>();
 *
 *
 * @return const YAML::Node& YAML Node
 */
static const YAML::Node& Params() { return ParamServer::Instance().Get(); }

}  // namespace tros_base
}  // namespace tros
