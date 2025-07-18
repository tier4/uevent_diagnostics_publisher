#include "uevent_diagnostics_publisher/interpteter_map.hpp"

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

TEST(InterpreterBaseTest, RegexTest)
{
  auto interpreter_map = uevent_diagnostics_publisher::InterpreterMap();
  auto vi_camera_interpreter = interpreter_map.getInterPreter("vi_camera");
  vi_camera_interpreter
    ->InterpreterBase::setup(  // call base class setup to bypass platform specific behavior
      ".*/tegra-capture-vi",   // dev_path
      "CAUSE1",                // identifier_key
      ".* 12-001c",            // key_regex
      "camera0",               // hardware_id
      "FUSA_HW_FAULT",         // value_key
      "bool");                 // value_type

  std::map<std::string, std::string> dummy_uevent;
  dummy_uevent["ACTION"] = "change";
  dummy_uevent["DEVPATH"] = "/devices/platform/tegra-capture-vi";
  dummy_uevent["SUBSYSTEM"] = "platform";
  dummy_uevent["FUSA_HW_FAULT"] = "1";
  dummy_uevent["CAUSE1"] = "tier4_isx021 12-001c";
  dummy_uevent["DRIVER"] = "tegra-camrtc-capture-vi";

  EXPECT_TRUE(vi_camera_interpreter->isTarget(dummy_uevent));

  std::map<std::string, std::string> dummy_uevent2;
  dummy_uevent2["ACTION"] = "change";
  dummy_uevent2["DEVPATH"] = "/devices/platform/tegra-capture-vi";
  dummy_uevent2["SUBSYSTEM"] = "platform";
  dummy_uevent2["FUSA_HW_FAULT"] = "1";
  dummy_uevent2["CAUSE1"] = "tier4_isx021 13-001c";
  dummy_uevent2["DRIVER"] = "tegra-camrtc-capture-vi";

  EXPECT_FALSE(vi_camera_interpreter->isTarget(dummy_uevent2));

  auto undefined_interpreter = interpreter_map.getInterPreter("undefined_hw");
  undefined_interpreter->setup(
    "no specified",   // dev_path
    "CAUSE1",         // identifier_key
    ".* 12-001c",     // key_regex
    "other hw",       // hardware_id
    "FUSA_HW_FAULT",  // value_key
    "bool");          // value_type
  EXPECT_FALSE(undefined_interpreter->isTarget(dummy_uevent));
}

TEST(ViCameraInterpreterTest, SearchVideoDeviceTest)
{
#ifdef V4L2_FILE_SEARCH_PATH
  std::cerr << "V4L2_FILE_SEARCH_PATH: " << V4L2_FILE_SEARCH_PATH << std::endl;
#endif

  auto interpreter_map = uevent_diagnostics_publisher::InterpreterMap();
  auto vi_camera_interpreter = interpreter_map.getInterPreter("vi_camera");
  vi_camera_interpreter
    ->InterpreterBase::setup(  // call base class setup to bypass platform specific behavior
      ".*/tegra-capture-vi",   // dev_path
      "CAUSE1",                // identifier_key
      ".* 12-001c",            // key_regex
      "camera0",               // hardware_id
      "FUSA_HW_FAULT",         // value_key
      "bool");                 // value_type
  auto interpreter =
    static_cast<uevent_diagnostics_publisher::ViCameraInterpreter *>(vi_camera_interpreter.get());

  EXPECT_EQ(
    interpreter->searchDeviceNodeFromI2cBusAddr(V4L2_FILE_SEARCH_PATH, ".* 12-001[a-z]"), "video0");
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
