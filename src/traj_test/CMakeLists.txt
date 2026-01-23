cmake_minimum_required(VERSION 3.8)
project(traj_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build environment option: REAL_HARDWARE or SITL
# Usage: colcon build --packages-select traj_test --cmake-args -DBUILD_ENV=SITL
# or: colcon build --packages-select traj_test --cmake-args -DBUILD_ENV=REAL_HARDWARE (default)
set(BUILD_ENV "REAL_HARDWARE" CACHE STRING "Build environment: REAL_HARDWARE or SITL")
set_property(CACHE BUILD_ENV PROPERTY STRINGS "REAL_HARDWARE" "SITL")

message(STATUS "==============================================")
message(STATUS "Build Environment: ${BUILD_ENV}")
message(STATUS "==============================================")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(offboard_state_machine REQUIRED)

include_directories(
  include
)

# Add compile definitions based on build environment
if(BUILD_ENV STREQUAL "SITL")
    add_compile_definitions(BUILD_SITL)
    message(STATUS "Compiling for SITL environment")
else()
    add_compile_definitions(BUILD_REAL_HARDWARE)
    message(STATUS "Compiling for REAL_HARDWARE environment")
endif()

add_executable(traj_test_node
  src/traj_test_node.cpp
  src/traj_test_main.cpp
)

ament_target_dependencies(traj_test_node
  rclcpp
  std_msgs
  px4_msgs
  geometry_msgs
  offboard_state_machine
)

install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  traj_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
