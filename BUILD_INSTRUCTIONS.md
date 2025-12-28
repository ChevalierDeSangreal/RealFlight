# RealFlight 编译说明

## 概述

RealFlight项目现在支持通过CMake参数在**实机环境**和**软件在环(SITL)环境**之间切换，无需维护两个独立的分支。

## 编译环境说明

### 1. 实机环境 (REAL_HARDWARE) - 默认

- **用途**: 在Radxa等实际硬件上部署
- **TensorFlow Lite路径**: `$HOME/Downloads/tensorflow/tflite_build`
- **Eigen3**: 使用 `eigen3_cmake_module`

### 2. 软件在环 (SITL)

- **用途**: 仿真测试和开发
- **TensorFlow Lite路径**: `$HOME/wangzimo/third_party/tensorflow_lite`
- **Eigen3**: 使用标准Eigen3包

## 编译命令

### 编译实机环境 (默认)

```bash
cd ~/wangzimo/RealFlight
colcon build
```

或者显式指定：

```bash
colcon build --cmake-args -DBUILD_ENV=REAL_HARDWARE
```

### 编译SITL环境

```bash
cd ~/wangzimo/RealFlight
colcon build --cmake-args -DBUILD_ENV=SITL
```

## 清理编译缓存

当在不同环境之间切换时，建议清理之前的编译缓存：

```bash
# 删除build和install目录
rm -rf build install

# 重新编译
colcon build --cmake-args -DBUILD_ENV=SITL  # 或 REAL_HARDWARE
```

## 受影响的包

以下包的CMakeLists.txt已经过修改以支持环境切换：

1. **hover_test** - 包含TensorFlow Lite依赖，支持两个版本（标准和50Hz）
2. **track_test** - 包含TensorFlow Lite依赖
3. **offboard_state_machine** - Eigen3依赖配置

其他包（如 `px4_msgs`, `px4_ros_com`等）不受影响。

## TensorFlow Lite依赖差异

### 实机环境 (REAL_HARDWARE)

使用构建目录结构，库文件位于：
- `${TFLITE_BUILD_DIR}/_deps/xnnpack-build/`
- `${TFLITE_BUILD_DIR}/_deps/flatbuffers-build/`
- `${TFLITE_BUILD_DIR}/_deps/ruy-build/ruy/`
- 等等

### SITL环境

使用已组织的库结构，所有库文件位于：
- `${TFLITE_LIB_DIR}/libXNNPACK.a`
- `${TFLITE_LIB_DIR}/libflatbuffers.a`
- `${TFLITE_LIB_DIR}/libruy_*.a`
- 等等

## 验证编译环境

编译时会在控制台输出当前的编译环境：

```
==============================================
Build Environment: REAL_HARDWARE
==============================================
```

或

```
==============================================
Build Environment: SITL
==============================================
```

## 故障排查

### 1. TensorFlow Lite库未找到

**错误信息**:
```
TFLite library not found: /path/to/libtensorflow-lite.a
```

**解决方法**:
- 确认TensorFlow Lite已正确安装在对应的路径
- 实机环境: `$HOME/Downloads/tensorflow/tflite_build`
- SITL环境: `$HOME/wangzimo/third_party/tensorflow_lite`

### 2. 环境切换后编译错误

**解决方法**:
```bash
rm -rf build install
colcon build --cmake-args -DBUILD_ENV=<你的环境>
```

### 3. Eigen3相关错误

**解决方法**:
- 确保安装了必要的依赖:
  ```bash
  sudo apt-get install libeigen3-dev
  sudo apt-get install ros-humble-eigen3-cmake-module  # 仅实机环境需要
  ```

## 自定义TensorFlow Lite路径

如果你的TensorFlow Lite安装在其他位置，可以在编译时指定：

```bash
colcon build --cmake-args \
  -DBUILD_ENV=SITL \
  -DTFLITE_ROOT=/your/custom/path/to/tensorflow_lite
```

或对于实机环境：

```bash
colcon build --cmake-args \
  -DBUILD_ENV=REAL_HARDWARE \
  -DTFLITE_BUILD_DIR=/your/custom/path/to/tflite_build \
  -DTFLITE_INCLUDE_DIR=/your/custom/path/to/tensorflow
```

## 迁移说明

如果你之前使用两个独立分支（RealFlight和RealFlightBackup），现在可以：

1. 只使用RealFlight分支
2. 根据需要使用不同的`BUILD_ENV`参数编译
3. 可以安全地删除或归档RealFlightBackup分支（建议先备份）

## 注意事项

1. 默认编译环境是`REAL_HARDWARE`，如需SITL环境必须显式指定
2. 切换环境时建议清理编译缓存以避免链接错误
3. 两个环境的TensorFlow Lite库结构不同，CMake会自动处理这些差异
4. 所有环境配置在CMakeLists.txt中集中管理，易于维护

## 技术细节

### CMake变量

- `BUILD_ENV`: 编译环境选择 (REAL_HARDWARE 或 SITL)
- `TFLITE_ROOT`: SITL环境的TFLite根目录
- `TFLITE_BUILD_DIR`: 实机环境的TFLite构建目录
- `TFLITE_INCLUDE_DIR`: TFLite头文件目录
- `TFLITE_LIB_DIR`: TFLite库文件目录

### 条件编译逻辑

CMakeLists.txt使用`if(BUILD_ENV STREQUAL "SITL")`来控制：
- TensorFlow Lite路径配置
- 库文件链接方式
- Eigen3的查找和链接方式

这确保了在不同环境下使用正确的依赖配置。

