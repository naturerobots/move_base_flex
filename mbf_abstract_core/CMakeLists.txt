cmake_minimum_required(VERSION 3.5)
project(mbf_abstract_core)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


ament_export_include_directories(include)
ament_export_dependencies(
  std_msgs
  geometry_msgs
)

install(DIRECTORY include/
DESTINATION include
)

ament_package()