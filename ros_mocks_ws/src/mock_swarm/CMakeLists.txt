cmake_minimum_required(VERSION 3.5)
project(mock_swarm)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ui_comm REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(far_gmapping src/mock_gmapping.cpp)
ament_target_dependencies(far_gmapping rclcpp)

add_executable(far_plan src/mock_plan.cpp)
ament_target_dependencies(far_plan rclcpp ui_comm ${dependencies} nlohmann_json)                      

target_link_libraries( far_plan
  nlohmann_json::nlohmann_json
)

install(TARGETS
  far_gmapping
  far_plan
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
