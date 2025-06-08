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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__PROFRAME_CAMERA_INTERPRETER_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__PROFRAME_CAMERA_INTERPRETER_HPP_

#include "uevent_diagnostics_publisher/interpreters/interpreter_base.hpp"

#include <optional>
#include <string>

namespace uevent_diagnostics_publisher
{
class ProFrameCameraInterpreter : public InterpreterBase
{
public:
  explicit ProFrameCameraInterpreter() { default_hardware_id_ = "proframe_camera"; }

  void getCurrentStatus(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

  void interpret(const UeventDataType & event) override;

protected:
  std::string observation_;
};

}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__PROFRAME_CAMERA_INTERPRETER_HPP_
