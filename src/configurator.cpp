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

#include "uevent_diagnostics_publisher/configurator.hpp"

#include <filesystem>

////////////////////////////////////////////////////////////////////////////////////////////////////
/// DEBUG
///
#include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////

namespace uevent_diagnostics_publisher
{
std::vector<UeventDescription> Configurator::parseYaml(const std::string & file_path)
{
  if (!std::filesystem::exists(file_path)) {
    throw std::runtime_error("Specified configuration yaml path is not available");
  }

  std::vector<UeventDescription> uevent_descriptions;

  YAML::Node doc = YAML::LoadFile(file_path);
  for (const auto & uevent_desc : doc) {  // top level nodes stand for one of `uevent_description`
    UeventDescription description;

    YAML::Node description_entries = uevent_desc.second;

    // Extract criteria
    YAML::Node criteria = description_entries["criteria"];
    for (const auto & entry : criteria) {
      const auto & criteria_key = entry.first.as<std::string>();
      const auto & criteria_val = entry.second.as<std::string>();
      description.criteria[criteria_key] = criteria_val;
    }

    // Exctract ramaining keys
    for (const auto & entry : description_entries) {
      const auto & desc_key = entry.first.as<std::string>();
      if (desc_key != "criteria") {
        const auto & desc_val = entry.second.as<std::string>();
        description.keys[desc_key] = desc_val;
      }
    }

    uevent_descriptions.push_back(description);
  }

  return uevent_descriptions;
}

std::shared_ptr<InterpreterBase> Configurator::registerInterpreter(
  const UeventDescription & description)
{
  // check input description includes mandatory keys
  auto key_check = [](const auto & target, const std::string & key) {
    if (target.find(key) == target.end()) {
      throw std::runtime_error("Invalud description is passed. Key '" + key + "' is mandatory");
    }
  };

  key_check(description.keys, "hardware_type");
  auto interpreter = InterpreterMap::getInterPreter(description.keys.at("hardware_type"));

  key_check(description.keys, "devpath");
  key_check(description.keys, "identifier_key");
  key_check(description.keys, "key_is");
  key_check(description.keys, "hardware_id");
  key_check(description.keys, "value_key");
  key_check(description.keys, "value_type");

  interpreter->setup(
    description.keys.at("devpath"), description.keys.at("identifier_key"),
    description.keys.at("key_is"), description.keys.at("hardware_id"),
    description.keys.at("value_key"), description.keys.at("value_type"));

  interpreter->installCriteriaToFilter(description.criteria);

  return interpreter;
}
}  // namespace uevent_diagnostics_publisher
