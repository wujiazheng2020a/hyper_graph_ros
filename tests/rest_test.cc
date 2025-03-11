/**
 * @file rest_test.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 *
 */

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "tros_base/proto/base_geometry.pb.h"
#include "tros_base/tros_base.h"

TEST(REST_TEST, MAIN) {
  tros::tros_base::TROSInit("../tests/data/param_file.yaml",
                            "../tests/data/data_graph_setting.yaml",
                            "../tests/data/network_config_test.yaml", 0);
}
