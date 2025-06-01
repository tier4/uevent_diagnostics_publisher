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

#ifndef UEVENT_DIAGNOSTICS_PUBLISHER__CONFIGURATOR_HPP_
#define UEVENT_DIAGNOSTICS_PUBLISHER__CONFIGURATOR_HPP_

#include "uevent_diagnostics_publisher/interpteter_map.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace uevent_diagnostics_publisher
{

struct UeventDescription
{
  std::unordered_map<std::string, std::string> keys;
  std::unordered_map<std::string, std::string> criteria;

  bool operator==(const UeventDescription & other) const
  {
    return (this->keys == other.keys) && (this->criteria == other.criteria);
  }

  friend std::ostream & operator<<(std::ostream & os, const UeventDescription & instance)
  {
    os << "Keys:" << std::endl;
    for (const auto & [k, v] : instance.keys) {
      os << "  " << k << ": " << v << std::endl;
    }
    os << "Criteria:" << std::endl;
    for (const auto & [k, v] : instance.criteria) {
      os << "  " << k << ": " << v << std::endl;
    }
    return os;
  }
};

class Configurator
{
public:
  explicit Configurator() {}

  /**
   * Parses a YAML file and extracts the data.
   *
   * @param file_path The path to the YAML file.
   *
   * Note:
   * uevent will come in seriese of <key>=<value> format.
   * The yaml file is expected to the sequence of the following blob:
   * - uevent_description:
   *   - hardware_type: <regular expression of hardware type to indicate special interpreter is
   * required>
   *   - devpath:  <devpath of uevent to identify which uevent interpreter will be used >
   *   - identifier_key: <key of the uevent to identify individual hardware>
   *   - key_is: <reglar expression of key contents to identify individual hardware>
   *   - hardware_id: <hardware_id to be used in diagnostics>
   *   - value_key: <key of the uevent to be used as a value of diagnostics>
   *   - value_type: <how can interpreters  recognize uevent value >
   *   - criteria: <criteria to judge the value is OK or ERROR>
   */
  static std::vector<UeventDescription> parseYaml(const std::string & file_path);

  static std::shared_ptr<InterpreterBase> registerInterpreter(const UeventDescription & descriptions);

};  // class Configurator
}  // namespace uevent_diagnostics_publisher

#endif  // UEVENT_DIAGNOSTICS_PUBLISHER__CONFIGURATOR_HPP_
