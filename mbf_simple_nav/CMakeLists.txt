cmake_minimum_required(VERSION 3.10)
project(mbf_simple_nav)

find_package(ament_cmake_ros REQUIRED)
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


add_library(${PROJECT_NAME}_lib
  src/simple_navigation_server.cpp
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${geometry_msgs_TARGETS}
  ${mbf_abstract_core_TARGETS}
  ${mbf_abstract_nav_TARGETS}
  ${mbf_msgs_TARGETS}
  ${mbf_simple_core_TARGETS}
  ${mbf_utility_TARGETS}
  ${nav_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME}_lib ALIAS ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_node
  src/simple_server_node.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}_lib
  ${geometry_msgs_TARGETS}
  ${mbf_abstract_core_TARGETS}
  ${mbf_abstract_nav_TARGETS}
  ${mbf_msgs_TARGETS}
  ${mbf_simple_core_TARGETS}
  ${mbf_utility_TARGETS}
  ${nav_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS 
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Add test lib with plugin(s)
  add_library(${PROJECT_NAME}_test_plugins
    test/plugins/test_planner.cpp
    test/plugins/test_plan_refiner.cpp
    test/plugins/test_controller.cpp
    test/plugins/test_recovery.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_plugins PUBLIC
    ${pluginlib_TARGETS}
    ${mbf_simple_core_TARGETS}
  )
  pluginlib_export_plugin_description_file(mbf_simple_core test/plugins/mbf_simple_nav_test_plugins.xml)
  install(TARGETS 
    ${PROJECT_NAME}_test_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # add gmock test suites
  find_package(ament_cmake_gmock REQUIRED)
  find_package(mbf_test_utility REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_integration_test test/simple_nav_integration_test.cpp)
  target_include_directories(${PROJECT_NAME}_integration_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_integration_test
    ${PROJECT_NAME}_lib
    ${mbf_test_utility_TARGETS}
    ${rclcpp_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

set(mbf_simple_nav_TARGETS mbf_simple_nav_lib)
ament_package()