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

#include "uevent_diagnostics_publisher/interpreters/interpreter_base.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <regex>
#include <string>

namespace
{
bool isEqualCaseInsensitive(const std::string & lhs, const std::string & rhs)
{
  return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), [](char a, char b) {
    return std::tolower(static_cast<unsigned char>(a)) ==
           std::tolower(static_cast<unsigned char>(b));
  });
}
}  // namespace

namespace uevent_diagnostics_publisher
{
void InterpreterBase::setup(
  const std::string & dev_path_regex, const std::string & identifier_key,
  const std::string & key_regex, const std::string & hardware_id, const std::string & value_key,
  const std::string & value_type)
{
  dev_path_str_ = dev_path_regex;
  dev_path_regex_ = std::regex(dev_path_str_);
  identifier_key_ = identifier_key;
  key_str_ = key_regex;
  key_regex_ = std::regex(key_str_);
  hardware_id_ = hardware_id;
  value_key_ = value_key;
  value_type_ = value_type;

  criteria_filter_ = ValueTypeMap::getValueType(value_type_);

  // Since uevent notifies only when a status change is detected by its nature,
  // set the default status as OK
  criteria_match_ = CriteriaMatches::OK;
}

void InterpreterBase::installCriteriaToFilter(
  const std::unordered_map<std::string, std::string> & criteria)
{
  criteria_filter_->setCriteria(criteria);
}

const std::string & InterpreterBase::getHardwareId() const
{
  return hardware_id_.empty() ? default_hardware_id_ : hardware_id_;
}

bool InterpreterBase::isTarget(const std::map<std::string, std::string> & event)
{
  // find dev_path entry in case insensitive manner
  auto dev_path_it = std::find_if(event.begin(), event.end(), [](const auto & pair) {
    return isEqualCaseInsensitive(pair.first, "devpath");
  });

  if (dev_path_it == event.end()) {
    return false;
  }

  // find identifier_key entry in case insensitive manner
  auto identifier_key_it = std::find_if(event.begin(), event.end(), [this](const auto & pair) {
    return isEqualCaseInsensitive(pair.first, identifier_key_);
  });

  if (identifier_key_it == event.end() && !identifier_key_.empty()) {
    return false;
  }

  // Once candidate entries are found, check the contents
  bool is_dev_path_match = std::regex_match(dev_path_it->second, dev_path_regex_);
  bool is_identifier_key_match = true;
  if (!identifier_key_.empty()) {
    is_identifier_key_match = std::regex_match(identifier_key_it->second, key_regex_);
  }

  return is_dev_path_match && is_identifier_key_match;
}

void InterpreterBase::getCurrentStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  switch (criteria_match_) {
    case CriteriaMatches::OK: {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
      break;
    }
    case CriteriaMatches::ERROR: {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "error detected");
      break;
    }
    default: {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "undefined thing may be input");
      break;
    }
  }
}

}  // namespace uevent_diagnostics_publisher
