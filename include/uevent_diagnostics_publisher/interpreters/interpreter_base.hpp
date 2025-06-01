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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__INTERPRETER_BASE_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__INTERPRETER_BASE_HPP_

#include "uevent_diagnostics_publisher/using_types.hpp"
#include "uevent_diagnostics_publisher/value_type_map.hpp"

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>

namespace uevent_diagnostics_publisher
{
class InterpreterBase
{
public:
  explicit InterpreterBase() { default_hardware_id_ = "default_hardware_id"; }
  virtual void setup(
    const std::string & dev_path_regex, const std::string & identifier_key,
    const std::string & key_regex, const std::string & hardware_id, const std::string & value_key,
    const std::string & value_type);
  void installCriteriaToFilter(const std::unordered_map<std::string, std::string> & criteria);
  const std::string & getHardwareId() const;
  bool isTarget(const UeventDataType & event);
  // NOTE: Since `stat_mutex_` is locked in `getCurrenetStatus`, derived classes must not lock the
  // mutex before calling InterpreterBase::getCurrentStatus
  virtual void getCurrentStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  virtual void interpret(const UeventDataType & event) { (void)event; };

protected:
  std::string dev_path_str_;
  std::regex dev_path_regex_;
  std::string identifier_key_;
  std::string key_str_;
  std::regex key_regex_;
  std::string hardware_id_;
  std::string value_key_;
  std::string value_type_;

  std::string default_hardware_id_;
  std::shared_ptr<ValueTypeBase> criteria_filter_;

  std::mutex stat_mutex_;
  CriteriaMatches criteria_match_;
};

}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__INTERPRETER_BASE_HPP_
