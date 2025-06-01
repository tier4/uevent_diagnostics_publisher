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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_DIAGNOSTICS_PUBLISHER_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_DIAGNOSTICS_PUBLISHER_HPP_

#include "uevent_diagnostics_publisher/using_types.hpp"
#include "uevent_diagnostics_publisher/configurator.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace uevent_diagnostics_publisher
{
class UeventDiagnosticsPublisher : public rclcpp::Node
{
public:
  explicit UeventDiagnosticsPublisher(const rclcpp::NodeOptions & options);
  ~UeventDiagnosticsPublisher();

protected:
  std::shared_ptr<std::thread> uevent_receiver_thread_;
  std::shared_ptr<std::thread> uevent_process_thread_;
  std::vector<std::shared_ptr<InterpreterBase>> interpreters_;
  std::vector<std::shared_ptr<diagnostic_updater::FunctionDiagnosticTask>> diagnostic_tasks_;
  std::vector<std::shared_ptr<diagnostic_updater::Updater>> diagnostic_updaters_;

  std::queue<UeventRawDataType> uevent_raw_data_queue_;
  std::mutex queue_mutex_;
  std::condition_variable uevent_ready_;
  bool stop_requested_;

  void receiveUevent();
  void processUevent();

};  // class UeventDiagnosticsPublisher
}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__UEVENT_DIAGNOSTICS_PUBLISHER_HPP_
