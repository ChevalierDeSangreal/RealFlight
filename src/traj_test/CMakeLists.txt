cmake_minimum_required(VERSION 3.8)
project(traj_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(offboard_state_machine REQUIRED)

# TensorFlow Lite
# Using local installation from Downloads/tensorflow/tflite_build
set(TFLITE_BUILD_DIR "$ENV{HOME}/Downloads/tensorflow/tflite_build" CACHE PATH "TensorFlow Lite build directory")
set(TFLITE_INCLUDE_DIR "$ENV{HOME}/Downloads/tensorflow" CACHE PATH "TensorFlow Lite include directory")
set(TFLITE_LIB_DIR "${TFLITE_BUILD_DIR}" CACHE PATH "TensorFlow Lite library directory")

# Check if TFLite is properly installed
if(NOT EXISTS ${TFLITE_INCLUDE_DIR}/tensorflow/lite)
    message(FATAL_ERROR "TFLite include directory not found: ${TFLITE_INCLUDE_DIR}/tensorflow/lite\n"
                        "Please ensure TensorFlow Lite is available at: ${TFLITE_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${TFLITE_LIB_DIR}/libtensorflow-lite.a)
    message(FATAL_ERROR "TFLite library not found: ${TFLITE_LIB_DIR}/libtensorflow-lite.a")
endif()

message(STATUS "TensorFlow Lite found:")
message(STATUS "  - Include: ${TFLITE_INCLUDE_DIR}")
message(STATUS "  - Library: ${TFLITE_LIB_DIR}/libtensorflow-lite.a")

include_directories(
  include
  ${TFLITE_INCLUDE_DIR}
)

add_executable(traj_test_node
  src/traj_test_node.cpp
  src/traj_test_main.cpp
)

ament_target_dependencies(traj_test_node
  rclcpp
  std_msgs
  px4_msgs
  offboard_state_machine
)

# Collect all necessary TFLite dependencies
file(GLOB ABSL_LIBS "${TFLITE_BUILD_DIR}/_deps/abseil-cpp-build/absl/*/*.a")
file(GLOB RUY_LIBS "${TFLITE_BUILD_DIR}/_deps/ruy-build/ruy/*.a")

# Link TensorFlow Lite and its dependencies
target_link_libraries(traj_test_node
  ${TFLITE_LIB_DIR}/libtensorflow-lite.a
  ${TFLITE_BUILD_DIR}/_deps/xnnpack-build/libXNNPACK.a
  ${TFLITE_BUILD_DIR}/_deps/flatbuffers-build/libflatbuffers.a
  ${TFLITE_BUILD_DIR}/pthreadpool/libpthreadpool.a
  ${TFLITE_BUILD_DIR}/_deps/cpuinfo-build/libcpuinfo.a
  ${TFLITE_BUILD_DIR}/_deps/farmhash-build/libfarmhash.a
  ${TFLITE_BUILD_DIR}/_deps/fft2d-build/libfft2d_fftsg.a
  ${TFLITE_BUILD_DIR}/_deps/fft2d-build/libfft2d_fftsg2d.a
  -Wl,--start-group
  ${RUY_LIBS}
  ${ABSL_LIBS}
  -Wl,--end-group
  ${CMAKE_DL_LIBS}
  pthread
  m  # Math library
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
