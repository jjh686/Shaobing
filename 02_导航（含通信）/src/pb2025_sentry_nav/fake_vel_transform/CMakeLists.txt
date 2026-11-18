cmake_minimum_required(VERSION 3.10)
project(fake_vel_transform)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fake_vel_transform.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fake_vel_transform::FakeVelTransform
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CLANG_FORMAT_VERSION 21)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Isolated link regression, independent of optional upstream lint tool versions.
option(SENTRY_LINK_TESTING "Build timestamped sentry source regression" OFF)
if(SENTRY_LINK_TESTING)
  find_package(tf2_msgs REQUIRED)
  enable_testing()
  add_executable(test_sentry_stamped test_sentry_stamped.cpp)
  target_link_libraries(test_sentry_stamped ${PROJECT_NAME})
  ament_target_dependencies(test_sentry_stamped rclcpp nav_msgs geometry_msgs tf2_msgs)
  add_test(NAME sentry_stamped_source COMMAND test_sentry_stamped)
  set_tests_properties(sentry_stamped_source PROPERTIES TIMEOUT 15
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}>:$ENV{LD_LIBRARY_PATH}")
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
