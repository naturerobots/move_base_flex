cmake_minimum_required(VERSION 3.5)
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


include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
   #src/navigation_utility.cpp
   #src/robot_information.cpp
   src/odometry_helper.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()