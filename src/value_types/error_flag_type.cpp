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

#include "uevent_diagnostics_publisher/value_types/error_flag_type.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace uevent_diagnostics_publisher
{
void ErrorFlagType::setCriteria(const std::unordered_map<std::string, std::string> & criteria)
{
  // Check the specific two keys are included in the provided criteria
  if (criteria.find(kKeyOk_) == criteria.end() || criteria.find(kKeyError_) == criteria.end()) {
    throw std::runtime_error(
      "Missing required criteria keys '" + kKeyOk_ + "' and '" + kKeyError_ + "' are mandatory");
  }

  // Extract values from the criteria map for use in setting the error flags
  ok_value_ = criteria.at(kKeyOk_);
  error_value_ = criteria.at(kKeyError_);
}

CriteriaMatches ErrorFlagType::applyCriteria(const std::vector<std::any> & values)
{
  if (values.size() != 1) {
    throw std::runtime_error("Incorrect number of values provided.");
  }

  const std::string & value = std::any_cast<std::string>(values[0]);

  return value == ok_value_      ? CriteriaMatches::OK
         : value == error_value_ ? CriteriaMatches::ERROR
                                 : CriteriaMatches::UNDEFINED;
}

}  // namespace uevent_diagnostics_publisher
