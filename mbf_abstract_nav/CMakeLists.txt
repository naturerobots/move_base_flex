cmake_minimum_required(VERSION 3.5)
project(mbf_abstract_nav)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mbf_msgs
  mbf_abstract_core
  mbf_utility
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  xmlrpcpp
  )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_abstract_core REQUIRED)
find_package(mbf_utility REQUIRED)


#maybe needed , maybe include in standard library
#find_package(Boost COMPONENTS thread chrono REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(MBF_ABSTRACT_SERVER_LIB mbf_abstract_server)

ament_export_include_directories(include) 

ament_export_libraries(${MBF_ABSTRACT_SERVER_LIB})

ament_target_dependencies(
  rclcpp
  rclcpp_action
  actionlib_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mbf_msgs
  mbf_abstract_core
  mbf_utility
  std_msgs
  std_srvs
  dynamic_reconfigure
)


add_library(${MBF_ABSTRACT_SERVER_LIB}
  src/controller_action.cpp
  src/planner_action.cpp
  src/recovery_action.cpp
  src/move_base_action.cpp
  src/abstract_execution_base.cpp
  src/abstract_navigation_server.cpp
  src/abstract_planner_execution.cpp
  src/abstract_controller_execution.cpp
  src/abstract_recovery_execution.cpp
)

install(TARGETS
  ${MBF_ABSTRACT_SERVER_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# TODO!!!

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   # gtests
#   catkin_add_gtest(${MBF_ABSTRACT_SERVER_LIB}_gtest test/abstract_execution_base.cpp)
#   target_link_libraries(${MBF_ABSTRACT_SERVER_LIB}_gtest ${MBF_ABSTRACT_SERVER_LIB})

#   # ros-tests
#   add_rostest_gmock(abstract_action_base_test
#     test/abstract_action_base.launch
#     test/abstract_action_base.cpp)
#   target_link_libraries(abstract_action_base_test ${MBF_ABSTRACT_SERVER_LIB})

#   add_rostest_gmock(abstract_controller_execution_test
#     test/abstract_controller_execution.launch
#     test/abstract_controller_execution.cpp)
#   target_link_libraries(abstract_controller_execution_test ${MBF_ABSTRACT_SERVER_LIB})

#   add_rostest_gmock(abstract_planner_execution_test
#     test/abstract_planner_execution.launch
#     test/abstract_planner_execution.cpp)
#   target_link_libraries(abstract_planner_execution_test ${MBF_ABSTRACT_SERVER_LIB})

#   add_rostest_gmock(planner_action_test
#     test/planner_action.launch
#     test/planner_action.cpp)
#   target_link_libraries(planner_action_test ${MBF_ABSTRACT_SERVER_LIB})
# endif()
