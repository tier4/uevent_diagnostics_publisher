cmake_minimum_required(VERSION 3.5)
project(uevent_diagnostics_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


file(GLOB_RECURSE CPP_SOURCES "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${CPP_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  SYSTEM
  rclcpp
  rclcpp_components
  diagnostic_updater
  diagnostic_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "uevent_diagnostics_publisher::UeventDiagnosticsPublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Utility node to display uevent contents
ament_auto_add_executable(uevent_printer
  src/utils/uevent_printer.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(unit_test_configurator
    test/test_configurator.cpp
  )

  target_link_libraries(unit_test_configurator
    ${PROJECT_NAME}
  )

  ament_add_gtest(unit_test_interpreter
    test/test_interpreter.cpp
  )

  target_link_libraries(unit_test_interpreter
    ${PROJECT_NAME}
  )

  ament_add_gtest(unit_test_value_type
    test/test_value_type.cpp
  )

  target_link_libraries(unit_test_value_type
    ${PROJECT_NAME}
  )

  # Copy test configuration file
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/test_config.yaml
    COPYONLY
  )

  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/test/video0/name
    ${CMAKE_CURRENT_BINARY_DIR}/video0/name
    COPYONLY
  )

  # Define macro to indicate test configuration file in absolute path
  target_compile_definitions(unit_test_configurator PRIVATE
    TEST_CONFIG_YAML_PATH="${CMAKE_CURRENT_BINARY_DIR}/test_config.yaml"
  )

  # Define macro to indicate test configuration file in absolute path
  target_compile_definitions(unit_test_interpreter PRIVATE
    V4L2_FILE_SEARCH_PATH="${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

ament_auto_package()
