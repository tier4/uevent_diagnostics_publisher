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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETER_MAP_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETER_MAP_HPP_

#include "uevent_diagnostics_publisher/interpreters/interpreter_base.hpp"
#include "uevent_diagnostics_publisher/interpreters/vi_camera_interpreter.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace uevent_diagnostics_publisher
{
class InterpreterMap
{
public:
  [[nodiscard]] static std::shared_ptr<InterpreterBase> getInterPreter(
    const std::string & hardware_type)
  {
    std::unordered_map<std::string, std::function<std::shared_ptr<InterpreterBase>()>>
      interpreters_map_ = {
        {"vi_camera", []() { return std::make_shared<ViCameraInterpreter>(); }},
        {"proframe_camera", []() { return std::make_shared<InterpreterBase>(); }},
      };

    auto it = interpreters_map_.find(hardware_type);
    if (it != interpreters_map_.end()) {
      return it->second();
    } else {
      return std::make_shared<InterpreterBase>();
    }
  }
};

}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETER_MAP_HPP_
