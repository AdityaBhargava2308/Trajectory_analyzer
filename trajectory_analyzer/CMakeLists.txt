cmake_minimum_required(VERSION 3.8)
project(trajectory_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(trajectory_analyzer
  "srv/SaveTrajectory.srv"
)

# Ensure service headers are available before compilation
ament_export_dependencies(rosidl_default_runtime)

# Executable: trajectory_writer
add_executable(trajectory_writer src/trajectory_writer.cpp)
target_include_directories(trajectory_writer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(trajectory_writer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(trajectory_writer
  rclcpp
  nav_msgs
  visualization_msgs
  rcpputils
  ament_index_cpp
)
rosidl_target_interfaces(trajectory_writer trajectory_analyzer rosidl_typesupport_cpp)

# Executable: trajectory_reader
add_executable(trajectory_reader src/trajectory_reader.cpp)
target_include_directories(trajectory_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(trajectory_reader PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(trajectory_reader
  rclcpp
  visualization_msgs
  ament_index_cpp
)
rosidl_target_interfaces(trajectory_reader trajectory_analyzer rosidl_typesupport_cpp)

# Install launch files
install(DIRECTORY
  launch
  rviz
  srv
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  trajectory_writer
  trajectory_reader
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
