cmake_minimum_required(VERSION 3.10)
project(mbf_utility)

find_package(ament_cmake REQUIRED)
set(dependencies
  geometry_msgs
  mbf_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/navigation_utility.cpp
  src/robot_information.cpp
  src/odometry_helper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  ${mbf_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

set(mbf_utility_TARGETS mbf_utility)
ament_package()