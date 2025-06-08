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

#include "uevent_diagnostics_publisher/interpreters/proframe_camera_interpreter.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <string>

namespace uevent_diagnostics_publisher
{
void ProFrameCameraInterpreter::interpret(const std::map<std::string, std::string> & event)
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  // Assume this interpreter only handles one field to watch the hardware status
  observation_ = event.at(value_key_);
  criteria_match_ = criteria_filter_->applyCriteria({observation_});
}

void ProFrameCameraInterpreter::getCurrentStatus(diagnostic_updater::DiagnosticStatusWrapper & stat) {
  {
    InterpreterBase::getCurrentStatus(stat);
    std::lock_guard<std::mutex> lock(stat_mutex_);
    stat.add("status", toString(criteria_match_));
    stat.add("obseravation", observation_);
  }
}

}  // namespace uevent_diagnostics_publisher
