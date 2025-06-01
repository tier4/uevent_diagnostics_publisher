#include "uevent_diagnostics_publisher/configurator.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace
{
// void printMap(const std::unordered_map<std::string, std::string> & map)
// {
//   // for (const auto & elem : map) {
//   //   //GTEST_LOG_(INFO) << elem.first << " : " << elem.second;
//   //   std::cout << elem.first << " : " << elem.second << std::endl;
//   // }
// }
}  // namespace

TEST(ConfiguratorTest, FileParse)
{
  //std::string file_path(TEST_CONFIG_YAML_PATH);  // TEST_CONFIG_YAML_PATH is defined in CMakeListx.txt
  std::string file_path("test_config.yaml");
  auto configurator = uevent_diagnostics_publisher::Configurator();
  const auto actual = configurator.parseYaml(file_path);
  for (const auto & desc : actual) {
    // // GTEST_LOG_(INFO) << "----------" << std::endl << "keys:";
    // std::cout << "----------" << std::endl << "keys:" << std::endl;
    // printMap(desc.keys);
    // // GTEST_LOG_(INFO) << "criteria:";
    // std::cout << "criteria:" << std::endl;
    // printMap(desc.criteria);
    std::cout << desc;
  }

  using uevent_diagnostics_publisher::UeventDescription;

  UeventDescription event1;
  event1.keys["hardware_type"] = "camera";
  event1.keys["devpath"] =  "/devices/platform/tegra-capture-vi";
  event1.keys["identifier_key"] = "CAUSE";
  event1.keys["key_is"] = ".* 12-001c";
  event1.keys["hardware_id"] = "camera0";
  event1.keys["value_key"] = "FUSA_HW_FAULT";
  event1.keys["value_type"] = "bool";
  event1.criteria["status_ok"] = "1";
  event1.criteria["status_error"] = "0";
  UeventDescription event2;
  event2.keys["hardware_type"] = "camera";
  event2.keys["devpath"] =  "/devices/platform/tegra-capture-vi";
  event2.keys["identifier_key"] = "CAUSE";
  event2.keys["key_is"] = "";
  event2.keys["hardware_id"] = "camera1";
  event2.keys["value_key"] = "FUSA_HW_FAULT";
  event2.keys["value_type"] = "bool";
  event2.criteria["status_ok"] = "1";
  event2.criteria["status_error"] = "0";

  std::vector<UeventDescription> desired = {event1, event2};
  for (size_t i = 0; i < actual.size(); i++) {
    // EXPECT_EQ(actual[i], desired[i]);
    EXPECT_EQ(actual[i].keys, desired[i].keys);
    EXPECT_EQ(actual[i].criteria, desired[i].criteria);
  }
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
