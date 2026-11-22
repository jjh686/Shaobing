cmake_minimum_required(VERSION 3.8)
project(rm2027_decision)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeammateState.msg" "msg/TacticalObjective.msg"
  "msg/DecisionState.msg" "msg/DecisionIntent.msg" "msg/NavigationEvidence.msg"
  DEPENDENCIES std_msgs builtin_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(rm2027_tree src/decision_node.cpp)
target_include_directories(rm2027_tree PRIVATE include)
ament_target_dependencies(rm2027_tree rclcpp rcl_interfaces behaviortree_cpp)
target_link_libraries(rm2027_tree "${cpp_typesupport_target}")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rm2027_tree PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(policy_test test/policy_test.cpp)
  target_include_directories(policy_test PRIVATE include)
  add_test(NAME rm2027_policy COMMAND policy_test)
  add_executable(freshness_test test/freshness_test.cpp)
  target_include_directories(freshness_test PRIVATE include)
  add_test(NAME rm2027_freshness COMMAND freshness_test)
  add_test(NAME rm2027_referee_fields
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/referee_rx_test.py")
  add_test(NAME rm2027_vision_feedback
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/vision_feedback_test.py")
endif()
install(TARGETS rm2027_tree DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/referee_state_adapter.py scripts/intent_nav_bridge.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/referee_rx.py scripts/vision_feedback.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config trees DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
