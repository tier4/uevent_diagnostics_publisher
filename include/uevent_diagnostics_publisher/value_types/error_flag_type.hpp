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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__ERROR_FLAG_TYPE_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__ERROR_FLAG_TYPE_HPP_

#include "uevent_diagnostics_publisher/value_types/value_type_base.hpp"

#include <string>

namespace uevent_diagnostics_publisher
{
class ErrorFlagType : public ValueTypeBase
{
public:
  explicit ErrorFlagType() {}
  void setCriteria(const std::unordered_map<std::string, std::string> & criteria) override;
  CriteriaMatches applyCriteria(const std::vector<std::any> & values) override;

 protected:
  inline static const std::string kKeyOk_ = "status_ok";
  inline static const std::string kKeyError_ = "status_error";
  std::string ok_value_;
  std::string error_value_;
};
}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__INTERPRETERS__ERROR_FLAG_TYPE_HPP_
