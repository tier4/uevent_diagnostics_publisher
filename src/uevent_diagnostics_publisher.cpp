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

#include "uevent_diagnostics_publisher/uevent_diagnostics_publisher.hpp"

#include "uevent_diagnostics_publisher/uevent_socket.hpp"

#include <map>
#include <memory>
#include <string>

namespace uevent_diagnostics_publisher
{
UeventDiagnosticsPublisher::UeventDiagnosticsPublisher(const rclcpp::NodeOptions & options)
: Node("uevent_diagnostics_publisher", options), stop_requested_(false)
{
  const auto & config_yaml_path = this->declare_parameter<std::string>("config_yaml_path", "");
  const auto diag_update_period = this->declare_parameter<double>("diag_update_period", 0.1);

  auto descriptions = Configurator::parseYaml(config_yaml_path);
  for (const auto & desc : descriptions) {
    // Construct interpreters according to each descriptions
    auto interpreter = Configurator::registerInterpreter(desc);

    // Set diagnostic task just copying each interpreters status
    auto diag_task = std::make_shared<diagnostic_updater::FunctionDiagnosticTask>(
      interpreter->getHardwareId() + "_uevent_diag",
      // pass a copy of the `interpreter`(shared_ptr to the interpreter) because it will be handled
      // in other threads run by diagnostic_updater ,and it needs to increase the reference pointer
      // of the shared pointer
      [interpreter](auto & stat) { interpreter->getCurrentStatus(stat); });

    // Construct & register diagnostic_updater related stuff
    auto diag_updater = std::make_shared<diagnostic_updater::Updater>(this);
    diag_updater->setPeriod(diag_update_period);
    diag_updater->setHardwareID(interpreter->getHardwareId());
    diag_updater->add(*diag_task);

    // save diagnostics_updater related instances for later use
    interpreters_.emplace_back(interpreter);
    diagnostic_tasks_.emplace_back(diag_task);
    diagnostic_updaters_.emplace_back(diag_updater);
  }

  // Run receiving thread separately to shorten response time
  uevent_receiver_thread_ = std::make_shared<std::thread>([this]() { this->receiveUevent(); });
  uevent_receiver_thread_->detach();

  uevent_process_thread_ = std::make_shared<std::thread>([this]() { this->processUevent(); });
}

UeventDiagnosticsPublisher::~UeventDiagnosticsPublisher()
{
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_requested_ = true;
    uevent_ready_.notify_one();
  }

  uevent_process_thread_->join();
}

void UeventDiagnosticsPublisher::receiveUevent()
{
  UeventSocket uevent_socket;
  while (true) {
    UeventRawDataType raw_uevent;
    try {
      if (stop_requested_) {
        return;
      }
      raw_uevent = uevent_socket.receiveUeventData();
      std::unique_lock<std::mutex> lock(queue_mutex_);
      uevent_raw_data_queue_.push(raw_uevent);
      uevent_ready_.notify_one();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      std::exit(EXIT_FAILURE);
    }
  }
}

void UeventDiagnosticsPublisher::processUevent()
{
  while (true) {
    UeventRawDataType uevent_raw_data;
    {  // pick up uevent data from queue
      std::unique_lock<std::mutex> lock(queue_mutex_);
      uevent_ready_.wait(lock, [this]() {
        // or wait until any data will be enqueued
        return !uevent_raw_data_queue_.empty() || stop_requested_;
      });
      if (stop_requested_ && uevent_raw_data_queue_.empty()) {
        return;
      }
      uevent_raw_data = std::move(uevent_raw_data_queue_.front());
      // uevent_raw_data = uevent_raw_data_queue_.front();
      uevent_raw_data_queue_.pop();
    }

    auto uevent_data = UeventSocket::createEnvMap(uevent_raw_data);

    for (size_t i = 0; i < interpreters_.size(); i++) {
      auto interpreter = interpreters_[i];
      auto diag_updater = diagnostic_updaters_[i];
      if (!interpreter->isTarget(uevent_data)) {
        // this uevent is unrelated to this interpreter
        continue;
      }

      interpreter->interpret(uevent_data);
      // Once the status updated, publish its change immediately
      diag_updater->force_update();
    }
  }
}

}  // namespace uevent_diagnostics_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uevent_diagnostics_publisher::UeventDiagnosticsPublisher)
