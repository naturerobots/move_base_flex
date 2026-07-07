cmake_minimum_required(VERSION 3.10)
project(mbf_abstract_core)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create an INTERFACE library target for this header-only library
add_library(${PROJECT_NAME} INTERFACE)

# Set include directories for the target
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link interface dependencies
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Install the target
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  std_msgs
  geometry_msgs
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(mbf_abstract_core_TARGETS mbf_abstract_core)
ament_package()