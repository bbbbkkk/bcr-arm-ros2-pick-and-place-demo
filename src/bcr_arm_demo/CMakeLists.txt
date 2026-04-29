cmake_minimum_required(VERSION 3.8)
project(bcr_arm_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(reach_target_demo src/reach_target_demo.cpp)

ament_target_dependencies(reach_target_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

install(TARGETS
  reach_target_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
