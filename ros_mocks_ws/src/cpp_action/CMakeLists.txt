cmake_minimum_required(VERSION 3.5)
project(cpp_action)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ui_comm REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# add_library(action_server SHARED
#   src/fibonacci_action_server.cpp)
# target_include_directories(action_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_server
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# ament_target_dependencies(action_server
#   "ui_comm"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components")
# rclcpp_components_register_node(action_server PLUGIN "cpp_action::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
# install(TARGETS
#   action_server
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

ament_package()
