cmake_minimum_required(VERSION 3.5)
project(mbf_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExePath.action"
  "action/GetPath.action"
  "action/MoveBase.action"
  "action/Recovery.action"
  "srv/CheckPath.srv"
  "srv/CheckPoint.srv"
  "srv/CheckPose.srv"
  "srv/FindValidPose.srv"
  "srv/SetTestRobotState.srv"
  DEPENDENCIES 
    std_msgs
    geometry_msgs
    nav_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
