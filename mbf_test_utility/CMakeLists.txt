cmake_minimum_required(VERSION 3.10)
project(mbf_test_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_sim
  src/robot_simulator.cpp
)
target_compile_features(${PROJECT_NAME}_sim PUBLIC
  c_std_99
  cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_sim
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${mbf_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# Create namespaced alias for modern CMake style
add_library(${PROJECT_NAME}::${PROJECT_NAME}_sim ALIAS ${PROJECT_NAME}_sim)

add_executable(${PROJECT_NAME}_sim_node
  src/robot_simulator_node.cpp
)
target_compile_features(${PROJECT_NAME}_sim_node PUBLIC
  c_std_99
  cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_sim_node
  ${PROJECT_NAME}_sim)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_sim
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2_ros)

set(mbf_test_utility_TARGETS mbf_test_utility_sim)
ament_package()
