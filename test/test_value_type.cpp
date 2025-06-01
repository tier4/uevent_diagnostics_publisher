#include "uevent_diagnostics_publisher/value_type_map.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

TEST(ValueTypeTest, ErrorFlagType)
{
  auto error_flag_type = uevent_diagnostics_publisher::ValueTypeMap::getValueType("error_flag");

  std::unordered_map<std::string, std::string> criteria = {
    {"status_ok", "0"},
    {"status_error", "1"},
  };

  error_flag_type->setCriteria(criteria);
  EXPECT_EQ(
    // error_flag_type->applyCriteria(std::vector<std::any>{std::string("0")}),
    error_flag_type->applyCriteria({std::string("0")}),
    uevent_diagnostics_publisher::CriteriaMatches::OK);
  EXPECT_EQ(
    error_flag_type->applyCriteria(std::vector<std::any>{std::string("1")}),
    uevent_diagnostics_publisher::CriteriaMatches::ERROR);
  EXPECT_EQ(
    error_flag_type->applyCriteria({std::string("2")}),
    uevent_diagnostics_publisher::CriteriaMatches::UNDEFINED);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
