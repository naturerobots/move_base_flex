cmake_minimum_required(VERSION 3.10)
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

add_library(mbf_abstract_server SHARED
  src/controller_action.cpp
  src/planner_action.cpp
  src/plan_refiner_action.cpp
  src/recovery_action.cpp
  src/move_base_action.cpp
  src/abstract_execution_base.cpp
  src/abstract_navigation_server.cpp
  src/abstract_planner_execution.cpp
  src/abstract_plan_refiner_execution.cpp
  src/abstract_controller_execution.cpp
  src/abstract_recovery_execution.cpp
)

target_include_directories(mbf_abstract_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mbf_abstract_server PUBLIC
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mbf_abstract_core_TARGETS}
  ${mbf_msgs_TARGETS}
  ${mbf_utility_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::mbf_abstract_server ALIAS mbf_abstract_server)

install(TARGETS mbf_abstract_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(mbf_abstract_server_gtest test/abstract_execution_base.cpp)
  target_include_directories(mbf_abstract_server_gtest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(mbf_abstract_server_gtest mbf_abstract_server)

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(abstract_action_base_test test/abstract_action_base.cpp)
  target_include_directories(abstract_action_base_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(abstract_action_base_test mbf_abstract_server)

  ament_add_gmock(abstract_controller_execution_test test/abstract_controller_execution.cpp)
  target_include_directories(abstract_controller_execution_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(abstract_controller_execution_test mbf_abstract_server)

  ament_add_gmock(abstract_planner_execution_test
    test/abstract_planner_execution.cpp)
  target_link_libraries(abstract_planner_execution_test mbf_abstract_server)

  ament_add_gmock(abstract_plan_refiner_execution_test
    test/abstract_plan_refiner_execution.cpp)
  target_link_libraries(abstract_plan_refiner_execution_test mbf_abstract_server)

  ament_add_gmock(planner_action_test
    test/planner_action.cpp)
  target_link_libraries(planner_action_test mbf_abstract_server)

  ament_add_gmock(abstract_plugin_manager_test
    test/abstract_plugin_manager.cpp)
  target_link_libraries(abstract_plugin_manager_test mbf_abstract_server)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

set(mbf_abstract_nav_TARGETS mbf_abstract_server)
ament_package()