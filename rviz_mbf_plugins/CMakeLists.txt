cmake_minimum_required(VERSION 3.10)
project(rviz_mbf_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(rviz_common REQUIRED)

# Now detect which Qt version rviz_common brought in
if(TARGET Qt6::Core)
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(QT_TARGET Qt6::Widgets)
  qt6_wrap_cpp(rviz_mbf_plugins_moc_files
    include/rviz_mbf_plugins/mbf_goal_actions_panel.hpp
  )
elseif(TARGET Qt5::Core) # humble compatibility
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_TARGET Qt5::Widgets)
  qt5_wrap_cpp(rviz_mbf_plugins_moc_files
    include/rviz_mbf_plugins/mbf_goal_actions_panel.hpp
  )
endif()

add_library(rviz_mbf_plugins SHARED
  ${rviz_mbf_plugins_moc_files}
  src/mbf_goal_actions_panel.cpp
)

target_include_directories(rviz_mbf_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rviz_mbf_plugins PUBLIC
  rclcpp::rclcpp
  ${rclcpp_action_TARGETS}
  ${pluginlib_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mbf_msgs_TARGETS}
  ${rviz_common_TARGETS}
  ${QT_TARGET}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS rviz_mbf_plugins)

target_compile_definitions(rviz_mbf_plugins PRIVATE "RVIZ_MBF_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS rviz_mbf_plugins
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

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(rviz_mbf_plugins_TARGETS rviz_mbf_plugins)
ament_package()