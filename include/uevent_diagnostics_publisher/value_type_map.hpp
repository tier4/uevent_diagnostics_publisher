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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__VALUE_TYPE_MAP_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__VALUE_TYPE_MAP_HPP_

#include "uevent_diagnostics_publisher/value_types/error_flag_type.hpp"
#include "uevent_diagnostics_publisher/value_types/value_type_base.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace uevent_diagnostics_publisher
{
class ValueTypeMap
{
public:
  explicit ValueTypeMap() {}
  [[nodiscard]] static std::shared_ptr<ValueTypeBase> getValueType(const std::string & value_type)
  {
    std::unordered_map<std::string, std::function<std::shared_ptr<ValueTypeBase>()>>
      value_type_map = {
        {"error_flag", []() { return std::make_shared<ErrorFlagType>(); }},
      };

    auto it = value_type_map.find(value_type);
    if (it != value_type_map.end()) {
      return it->second();
    } else {
      return std::make_shared<ValueTypeBase>();
    }
  }
};

}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__VALUE_TYPE_MAP_HPP_
