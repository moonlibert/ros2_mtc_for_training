cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# Level 1: 最小任务 —— CurrentState + MoveTo
add_executable(l1_minimal_task src/l1_minimal_task.cpp)
ament_target_dependencies(l1_minimal_task rclcpp moveit_task_constructor_core)

install(TARGETS l1_minimal_task
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
