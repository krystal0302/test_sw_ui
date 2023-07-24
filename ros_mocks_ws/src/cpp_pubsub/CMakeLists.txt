cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ui_comm REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
    include
    nlohmann_json
)

add_executable(talker src/mini_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs ui_comm)

add_executable(listener src/mini_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs ui_comm)

# add_executable(amr_detector src/amr_detection_mock.cpp)
# ament_target_dependencies(amr_detector rclcpp std_msgs geometry_msgs ui_comm)

# add_executable(mock_flow src/flow_mock.cpp)
# ament_target_dependencies(mock_flow rclcpp std_msgs geometry_msgs ui_comm)

add_executable(mock_plan src/mini_service.cpp)
ament_target_dependencies(mock_plan rclcpp ui_comm ${dependencies} nlohmann_json)                      

target_link_libraries( mock_plan
  nlohmann_json::nlohmann_json
)

add_executable(client src/mini_client.cpp)
ament_target_dependencies(client rclcpp ui_comm)                      


install(TARGETS
  talker
  listener
  # amr_detector
  # mock_flow
  mock_plan
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
