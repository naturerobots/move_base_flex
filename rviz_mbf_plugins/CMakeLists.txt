cmake_minimum_required(VERSION 3.8)
project(rviz_mbf_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Enable Qt MOC automatically
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(rviz_common REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME}_mbf_goal_actions_panel
  src/mbf_goal_actions_panel.cpp)

target_include_directories(${PROJECT_NAME}_mbf_goal_actions_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_mbf_goal_actions_panel
  rclcpp
  rclcpp_action
  pluginlib
  geometry_msgs
  mbf_msgs
  rviz_common
)

target_link_libraries(${PROJECT_NAME}_mbf_goal_actions_panel
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}_mbf_goal_actions_panel
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/ 
  DESTINATION include
)

install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
