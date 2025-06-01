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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__VALUE_TYPE_BASE_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__VALUE_TYPE_BASE_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <any>
#include <unordered_map>
#include <vector>
#include <string>

namespace uevent_diagnostics_publisher
{
enum class CriteriaMatches {
  OK = diagnostic_msgs::msg::DiagnosticStatus::OK,
  WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN,
  ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR,
  STALE = diagnostic_msgs::msg::DiagnosticStatus::STALE,
  UNDEFINED,
};

inline const std::string toString(CriteriaMatches c) {
  switch (c) {
    case CriteriaMatches::OK: return "OK";
    case CriteriaMatches::WARN: return "WARN" ;
    case CriteriaMatches::ERROR:  return "ERROR";
    case CriteriaMatches::STALE: return "STALE";
    case CriteriaMatches::UNDEFINED:
    default: return "UNDEFINED";
  }
}

class ValueTypeBase
{
public:
  explicit ValueTypeBase() {}

  virtual void setCriteria(const std::unordered_map<std::string, std::string> & criteria)
  {
    (void)criteria;
  }

  [[nodiscard]] virtual CriteriaMatches applyCriteria(const std::vector<std::any> & values)
  {
    (void)values;
    return CriteriaMatches::UNDEFINED;
  }
};
}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__VALUE_TYPE_BASE_HPP_
