cmake_minimum_required(VERSION 3.10)
project(mbf_simple_core)

find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp
  geometry_msgs
  mbf_abstract_core
  mbf_utility
)

foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

# Create INTERFACE library target
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${rclcpp_TARGETS}
  ${mbf_utility_TARGETS}
  ${mbf_abstract_core_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

set(mbf_simple_core_TARGETS mbf_simple_core)
ament_package()