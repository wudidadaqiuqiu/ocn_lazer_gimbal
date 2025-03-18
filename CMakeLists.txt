cmake_minimum_required(VERSION 3.8)
project(ocn_lazer_gimbal)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_msg REQUIRED)
find_package(embedded_interface REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  robot_msg
  embedded_interface
)

add_executable(test1
  src/gimbal.cpp)
target_include_directories(test1 PRIVATE include)
ament_target_dependencies(test1 ${dependencies})

install (TARGETS
  test1
DESTINATION lib/${PROJECT_NAME})

ament_package()
