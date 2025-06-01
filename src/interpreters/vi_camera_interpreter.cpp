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

#include "uevent_diagnostics_publisher/interpreters/vi_camera_interpreter.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <string>

////////////////////////////////////////////////////////////////////////////////////////////////////
/// DEBUG
// #include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////

namespace uevent_diagnostics_publisher
{
void ViCameraInterpreter::setup(
  const std::string & dev_path_regex, const std::string & identifier_key,
  const std::string & key_regex, const std::string & hardware_id, const std::string & value_key,
  const std::string & value_type)
{
  InterpreterBase::setup(
    dev_path_regex, identifier_key, key_regex, hardware_id, value_key, value_type);

  // find correspondign device node name
  std::regex i2c_dev_addr_pattern("[0-9]+-[0-9]+[a-zA-Z]");  // will match strings like "12-001c"
  std::smatch match;
  std::string i2c_dev_addr_str;
  if (std::regex_search(key_str_, match, i2c_dev_addr_pattern)) {
    i2c_dev_addr_str = match.str();
  } else {
    throw std::runtime_error("I2C bus and address pattern is not found in the key");
  }

  device_node_ = searchDeviceNodeFromI2cBusAddr("/sys/class/video4linux", i2c_dev_addr_str);
}

std::optional<std::string> ViCameraInterpreter::searchDeviceNodeFromI2cBusAddr(
  const std::string & search_path, const std::string & i2c_bus_addr)
{
  // Search "<search_path>/videoX/name" contents if `i2c_bus_addr` is included in it
  // If yes, `videoX` (e.g., `video0`) is the device node corresponding to the `i2c_bud_addr` (e.g.,
  // `12-001c`)
  std::optional<std::string> device_node_name = std::nullopt;
  for (const auto & entry : std::filesystem::directory_iterator(search_path)) {
    if (!entry.is_directory()) {
      continue;
    }

    std::string video_name = entry.path().filename();
    std::regex v4l2_device_node_pattern("video[0-9]+$");
    if (!std::regex_search(video_name, v4l2_device_node_pattern)) {
      continue;
    }

    std::filesystem::path name_file = entry.path() / "name";
    if (!std::filesystem::exists(name_file)) {
      continue;
    }

    std::ifstream file(name_file);
    if (!file) {
      continue;
    }

    std::string line;
    while (std::getline(file, line)) {
      // Look for the pattern "bus-addr" in the line
      std::regex i2c_pattern("[0-9]+-[0-9]+[a-zA-Z]+");
      std::smatch match;
      if (std::regex_search(line, match, i2c_pattern)) {
        if (match.str() == i2c_bus_addr) {
          device_node_name = video_name;
        }
      }
    }
  }
  return device_node_name;
}

void ViCameraInterpreter::interpret(const std::map<std::string, std::string> & event)
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  // Assume this interpreter only handles one field to watch the hardware status
  observation_ = event.at(value_key_);
  criteria_match_ = criteria_filter_->applyCriteria({observation_});
}

void ViCameraInterpreter::getCurrentStatus(diagnostic_updater::DiagnosticStatusWrapper & stat) {
  {
    InterpreterBase::getCurrentStatus(stat);
    std::lock_guard<std::mutex> lock(stat_mutex_);
    stat.add("status", toString(criteria_match_));
    stat.add("obseravation", observation_);
    stat.add("device_node", device_node_.value_or(""));
  }
}

}  // namespace uevent_diagnostics_publisher
