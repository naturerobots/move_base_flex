cmake_minimum_required(VERSION 3.5)
project(mbf_simple_core)

find_package(ament_cmake REQUIRED)
set(dependencies
 std_msgs
 geometry_msgs
 mbf_abstract_core
 mbf_utility
 tf2
)

foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()