cmake_minimum_required(VERSION 3.5)
project(logging_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(far_task_msgs REQUIRED)
find_package(far_fleet_msgs REQUIRED)
find_package(far_plan_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(yaml_cpp_vendor REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(SQLite3)

include_directories(${LIBXML2_INCLUDE_DIRS})
link_libraries(${LIBXML2_LIBRARY})

add_executable(${PROJECT_NAME} src/logging_core.cpp)

# target_include_directories(${PROJECT_NAME}
#         PRIVATE
#         src
#         )
include_directories(${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs far_task_msgs far_fleet_msgs far_plan_msgs yaml_cpp_vendor)
include_directories(${SQLITE3_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} xml2)
# target_link_libraries (${PROJECT_NAME} boost_filesystem)
target_link_libraries(${PROJECT_NAME} sqlite3)

#------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND FALSE)

  find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
endif()

ament_package()

# Generate version control information for binary package
set(VERSION_CONTROL ${CMAKE_INSTALL_PREFIX}/DEBIAN)
add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
add_definitions("-D${PROJECT_NAME}_MAINTAINER=\"${${PROJECT_NAME}_MAINTAINER}\"")
execute_process(
    COMMAND bash -c "mkdir -p '${VERSION_CONTROL}'"
    COMMAND bash -c "touch '${VERSION_CONTROL}'/control"
    COMMAND bash -c "echo 'Package: ${PROJECT_NAME}' | tr _ - > '${VERSION_CONTROL}'/control"
)
execute_process(
    COMMAND bash -c "echo 'Architecture: all' >> '${VERSION_CONTROL}'/control"
    COMMAND bash -c "echo 'Version: ${${PROJECT_NAME}_VERSION}' >> '${VERSION_CONTROL}'/control"
    COMMAND bash -c "echo 'Maintainer: ${${PROJECT_NAME}_MAINTAINER}' >> '${VERSION_CONTROL}'/control"
    COMMAND bash -c "echo 'Description: ${PROJECT_NAME} package release' >> '${VERSION_CONTROL}'/control"
)