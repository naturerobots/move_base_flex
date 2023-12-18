cmake_minimum_required(VERSION 3.5)
project(mbf_abstract_nav)

find_package(ament_cmake REQUIRED)
set(dependencies
  geometry_msgs
  mbf_abstract_core
  mbf_msgs
  mbf_utility
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

# TODO install python via ament_cmake
#ament_python_install_package(${PROJECT_NAME})

set(MBF_ABSTRACT_SERVER_LIB mbf_abstract_server)
add_library(${MBF_ABSTRACT_SERVER_LIB} SHARED
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
ament_target_dependencies(${MBF_ABSTRACT_SERVER_LIB} ${dependencies})

install(TARGETS ${MBF_ABSTRACT_SERVER_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${MBF_ABSTRACT_SERVER_LIB}_gtest test/abstract_execution_base.cpp)
  target_include_directories(${MBF_ABSTRACT_SERVER_LIB}_gtest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${MBF_ABSTRACT_SERVER_LIB}_gtest ${MBF_ABSTRACT_SERVER_LIB})

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(abstract_action_base_test test/abstract_action_base.cpp)
  target_include_directories(abstract_action_base_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(abstract_action_base_test ${MBF_ABSTRACT_SERVER_LIB})

  ament_add_gmock(abstract_controller_execution_test test/abstract_controller_execution.cpp)
  target_include_directories(abstract_controller_execution_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(abstract_controller_execution_test ${MBF_ABSTRACT_SERVER_LIB})

  ament_add_gmock(abstract_planner_execution_test
    test/abstract_planner_execution.cpp)
  target_link_libraries(abstract_planner_execution_test ${MBF_ABSTRACT_SERVER_LIB})

  ament_add_gmock(planner_action_test
    test/planner_action.cpp)
  target_link_libraries(planner_action_test ${MBF_ABSTRACT_SERVER_LIB})
endif()

ament_export_include_directories(include) 
ament_export_libraries(${MBF_ABSTRACT_SERVER_LIB})
ament_export_dependencies(${dependencies})
ament_package()