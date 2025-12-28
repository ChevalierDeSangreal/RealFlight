cmake_minimum_required(VERSION 3.8)
project(offboard_state_machine)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Build environment option: REAL_HARDWARE or SITL
# Usage: colcon build --cmake-args -DBUILD_ENV=SITL
# or: colcon build --cmake-args -DBUILD_ENV=REAL_HARDWARE (default)
set(BUILD_ENV "REAL_HARDWARE" CACHE STRING "Build environment: REAL_HARDWARE or SITL")
set_property(CACHE BUILD_ENV PROPERTY STRINGS "REAL_HARDWARE" "SITL")

# Handle Eigen3 dependency based on build environment
if(BUILD_ENV STREQUAL "SITL")
    # SITL environment: Use standard Eigen3 package
    find_package(Eigen3 REQUIRED)
else()
    # REAL_HARDWARE environment: Use eigen3_cmake_module
    find_package(eigen3_cmake_module REQUIRED)
    find_package(Eigen3 REQUIRED)
endif()

# ---------- build target ----------
add_executable(offboard_fsm_node
  src/offboard_fsm_node.cpp
  src/main.cpp
)

target_compile_features(offboard_fsm_node PUBLIC cxx_std_17)

# Handle Eigen3 include directories based on build environment
if(BUILD_ENV STREQUAL "SITL")
    # SITL environment: Explicitly add Eigen3 include directories
    target_include_directories(offboard_fsm_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${EIGEN3_INCLUDE_DIRS}
    )
else()
    # REAL_HARDWARE environment: Use standard include directories
    target_include_directories(offboard_fsm_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
endif()

ament_target_dependencies(offboard_fsm_node
  rclcpp
  std_msgs
  px4_msgs
)

# Handle Eigen3 linking based on build environment
if(BUILD_ENV STREQUAL "SITL")
    # SITL environment: Add Eigen3 as ament dependency
    ament_target_dependencies(offboard_fsm_node Eigen3)
else()
    # REAL_HARDWARE environment: Link Eigen3 library
    target_link_libraries(offboard_fsm_node
      Eigen3::Eigen
    )
endif()

# ---------- install ----------
install(TARGETS offboard_fsm_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers so other packages can use them
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export include directories for downstream packages
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs px4_msgs Eigen3)

ament_package()