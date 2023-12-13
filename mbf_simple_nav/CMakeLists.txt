cmake_minimum_required(VERSION 3.5)
project(mbf_simple_nav)

find_package(ament_cmake REQUIRED)
set(dependencies
  geometry_msgs
  mbf_abstract_core
  mbf_abstract_nav
  mbf_msgs
  mbf_simple_core
  mbf_utility
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_ros
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

set(MBF_SIMPLE_SERVER_LIB mbf_simple_server)
add_library(${MBF_SIMPLE_SERVER_LIB}
  src/simple_navigation_server.cpp
)
ament_target_dependencies(${MBF_SIMPLE_SERVER_LIB} ${dependencies})

set(MBF_SIMPLE_SERVER_NODE mbf_simple_nav)
add_executable(${MBF_SIMPLE_SERVER_NODE} src/simple_server_node.cpp)
target_link_libraries(${MBF_SIMPLE_SERVER_NODE} ${MBF_SIMPLE_SERVER_LIB})
ament_target_dependencies(${MBF_SIMPLE_SERVER_NODE} ${dependencies})

install(TARGETS
  ${MBF_SIMPLE_SERVER_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
 ${MBF_SIMPLE_SERVER_NODE}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${MBF_SIMPLE_SERVER_LIB})
ament_export_dependencies(${dependencies})
ament_package()