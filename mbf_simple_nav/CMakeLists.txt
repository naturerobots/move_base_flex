cmake_minimum_required(VERSION 3.5)
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
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/simple_server_node.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

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
    test/plugins/test_controller.cpp
    test/plugins/test_recovery.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_plugins pluginlib mbf_simple_core)
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
  ament_target_dependencies(${PROJECT_NAME}_integration_test mbf_test_utility rclcpp)
  target_link_libraries(${PROJECT_NAME}_integration_test ${PROJECT_NAME}_lib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()