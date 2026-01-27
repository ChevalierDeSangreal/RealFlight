cmake_minimum_required(VERSION 3.8)
project(track_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(offboard_state_machine REQUIRED)

# Build environment option: REAL_HARDWARE or SITL
# Usage: colcon build --cmake-args -DBUILD_ENV=SITL
# or: colcon build --cmake-args -DBUILD_ENV=REAL_HARDWARE (default)
set(BUILD_ENV "REAL_HARDWARE" CACHE STRING "Build environment: REAL_HARDWARE or SITL")
set_property(CACHE BUILD_ENV PROPERTY STRINGS "REAL_HARDWARE" "SITL")

message(STATUS "==============================================")
message(STATUS "Build Environment: ${BUILD_ENV}")
message(STATUS "==============================================")

# TensorFlow Lite configuration based on build environment
if(BUILD_ENV STREQUAL "SITL")
    # SITL environment: Using third_party installation
    set(TFLITE_ROOT "$ENV{HOME}/wangzimo/third_party/tensorflow_lite" CACHE PATH "TensorFlow Lite root directory")
    set(TFLITE_INCLUDE_DIR "${TFLITE_ROOT}/include" CACHE PATH "TensorFlow Lite include directory")
    set(TFLITE_LIB_DIR "${TFLITE_ROOT}/lib" CACHE PATH "TensorFlow Lite library directory")
else()
    # REAL_HARDWARE environment: Using Downloads installation
    set(TFLITE_BUILD_DIR "$ENV{HOME}/Downloads/tensorflow/tflite_build" CACHE PATH "TensorFlow Lite build directory")
    set(TFLITE_INCLUDE_DIR "$ENV{HOME}/Downloads/tensorflow" CACHE PATH "TensorFlow Lite include directory")
    set(TFLITE_LIB_DIR "${TFLITE_BUILD_DIR}" CACHE PATH "TensorFlow Lite library directory")
endif()

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

# Add include directories based on build environment
if(BUILD_ENV STREQUAL "SITL")
    include_directories(
      include
      ${TFLITE_INCLUDE_DIR}
    )
else()
    # REAL_HARDWARE environment: Add FlatBuffers and other dependency headers
    include_directories(
      include
      ${TFLITE_INCLUDE_DIR}
      ${TFLITE_BUILD_DIR}/flatbuffers/include
      ${TFLITE_BUILD_DIR}/abseil-cpp
      ${TFLITE_BUILD_DIR}/ruy
    )
endif()

add_executable(track_test_node
  src/track_test_node.cpp
  src/track_test_main.cpp
  src/target_generator.cpp
)

ament_target_dependencies(track_test_node
  rclcpp
  std_msgs
  geometry_msgs
  px4_msgs
  offboard_state_machine
)

# Target publisher node (for publishing target object circular trajectory)
add_executable(target_publisher_node
  src/target_publisher_node.cpp
  src/target_publisher_main.cpp
)

ament_target_dependencies(target_publisher_node
  rclcpp
  geometry_msgs
)

# Vicon to target converter node (for converting Vicon topics to target topics)
add_executable(vicon_to_target_node
  src/vicon_to_target_node.cpp
  src/vicon_to_target_main.cpp
)

ament_target_dependencies(vicon_to_target_node
  rclcpp
  geometry_msgs
)

# Link TensorFlow Lite and its dependencies based on build environment
if(BUILD_ENV STREQUAL "SITL")
    # SITL environment: Using organized library structure
    file(GLOB ABSL_LIBS "${TFLITE_LIB_DIR}/libabsl_*.a")
    
    target_link_libraries(track_test_node
      ${TFLITE_LIB_DIR}/libtensorflow-lite.a
      ${TFLITE_LIB_DIR}/libXNNPACK.a
      ${TFLITE_LIB_DIR}/libruy_frontend.a
      ${TFLITE_LIB_DIR}/libruy_ctx.a
      ${TFLITE_LIB_DIR}/libruy_context.a
      ${TFLITE_LIB_DIR}/libruy_context_get_ctx.a
      ${TFLITE_LIB_DIR}/libruy_trmul.a
      ${TFLITE_LIB_DIR}/libruy_prepare_packed_matrices.a
      ${TFLITE_LIB_DIR}/libruy_pack_avx512.a
      ${TFLITE_LIB_DIR}/libruy_pack_avx2_fma.a
      ${TFLITE_LIB_DIR}/libruy_pack_avx.a
      ${TFLITE_LIB_DIR}/libruy_pack_arm.a
      ${TFLITE_LIB_DIR}/libruy_kernel_avx512.a
      ${TFLITE_LIB_DIR}/libruy_kernel_avx2_fma.a
      ${TFLITE_LIB_DIR}/libruy_kernel_avx.a
      ${TFLITE_LIB_DIR}/libruy_kernel_arm.a
      ${TFLITE_LIB_DIR}/libruy_thread_pool.a
      ${TFLITE_LIB_DIR}/libruy_blocking_counter.a
      ${TFLITE_LIB_DIR}/libruy_wait.a
      ${TFLITE_LIB_DIR}/libruy_allocator.a
      ${TFLITE_LIB_DIR}/libruy_apply_multiplier.a
      ${TFLITE_LIB_DIR}/libruy_block_map.a
      ${TFLITE_LIB_DIR}/libruy_cpuinfo.a
      ${TFLITE_LIB_DIR}/libruy_denormal.a
      ${TFLITE_LIB_DIR}/libruy_have_built_path_for_avx512.a
      ${TFLITE_LIB_DIR}/libruy_have_built_path_for_avx2_fma.a
      ${TFLITE_LIB_DIR}/libruy_have_built_path_for_avx.a
      ${TFLITE_LIB_DIR}/libruy_prepacked_cache.a
      ${TFLITE_LIB_DIR}/libruy_profiler_instrumentation.a
      ${TFLITE_LIB_DIR}/libruy_profiler_profiler.a
      ${TFLITE_LIB_DIR}/libruy_system_aligned_alloc.a
      ${TFLITE_LIB_DIR}/libruy_tune.a
      ${TFLITE_LIB_DIR}/libflatbuffers.a
      ${TFLITE_LIB_DIR}/libeight_bit_int_gemm.a
      ${TFLITE_LIB_DIR}/libfft2d_fftsg.a
      ${TFLITE_LIB_DIR}/libfft2d_fftsg2d.a
      ${TFLITE_LIB_DIR}/libcpuinfo.a
      ${TFLITE_LIB_DIR}/libpthreadpool.a
      ${TFLITE_LIB_DIR}/libfarmhash.a
      ${CMAKE_DL_LIBS}
      pthread
      m  # Math library
      -Wl,--start-group
      ${ABSL_LIBS}
      -Wl,--end-group
    )
else()
    # REAL_HARDWARE environment: Using build directory structure
    file(GLOB ABSL_LIBS "${TFLITE_BUILD_DIR}/_deps/abseil-cpp-build/absl/*/*.a")
    file(GLOB RUY_LIBS "${TFLITE_BUILD_DIR}/_deps/ruy-build/ruy/*.a")
    
    target_link_libraries(track_test_node
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
endif()

install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  track_test_node
  target_publisher_node
  vicon_to_target_node
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

# Install scripts
install(PROGRAMS
  scripts/publish_target.py
  DESTINATION lib/${PROJECT_NAME}
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
