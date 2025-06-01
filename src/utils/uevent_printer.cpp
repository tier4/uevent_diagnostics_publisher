// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "uevent_diagnostics_publisher/uevent_socket.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

/**
 * @brief Simple utility function just dipslaying uevent contents
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("uevent_printer");

  uevent_diagnostics_publisher::UeventSocket uevent_socket;

  std::cout << "Uevent Printer started. Waiting uevent..." << std::endl;
  std::cout << "----------" << std::endl;

  while (rclcpp::ok()) {
    uevent_diagnostics_publisher::UeventRawDataType uevent_raw_data;
    try {
      uevent_raw_data = uevent_socket.receiveUeventData();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "%s", e.what());
      std::exit(EXIT_FAILURE);
    }

    auto uevent_list = uevent_diagnostics_publisher::UeventSocket::createEnvMap(uevent_raw_data);

    for (const auto & [k, v] : uevent_list) {
      std::cout << "ENV: key=" << k << ", value=" << v << std::endl;
    }
    std::cout << "----------" << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
