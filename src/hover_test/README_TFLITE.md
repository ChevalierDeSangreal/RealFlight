# TFLite Neural Network Control for hover_test

本文档说明如何使用 TensorFlow Lite 神经网络模型控制无人机悬停在目标点。

## 概述

`hover_test` 包已集成 TFLite 推理功能，使用训练好的神经网络策略来生成四旋翼控制指令，使无人机悬停在指定目标点。

### 模型信息

- **模型文件**: `config/hoverVer0_policy.tflite`
- **输入维度**: 130 (10个历史步 × 13维/步，每步包含 [action(4) + obs(9)])
- **输出维度**: 4 (thrust, omega_x, omega_y, omega_z)
- **观测空间** (9维，机体系坐标):
  - 机体系速度 (3): v_body_x, v_body_y, v_body_z
  - 机体系重力方向 (3): g_body_x, g_body_y, g_body_z
  - 机体系到目标点距离 (3): target_distance_body_x, target_distance_body_y, target_distance_body_z

### 控制流程

1. 每个控制周期 (100Hz, dt=0.01s) 获取当前观测 (9维)
2. 观测转换为机体系坐标：速度、重力方向、到目标点的距离
3. TFLite策略维护10步历史缓冲区，每步包含 [action(4), obs(9)]
4. 神经网络推理输出控制指令（体轴角速率 + 推力）
5. 动作更新：10Hz进行神经网络推理，100Hz发送控制指令
6. 发布到 PX4 的 `VehicleRatesSetpoint` 话题

## 编译前准备

### 1. 安装 TensorFlow Lite

```bash
# 方式1: 使用预编译库（推荐）
# 下载 TensorFlow Lite C++ 库
wget https://github.com/tensorflow/tensorflow/releases/download/v2.x.x/libtensorflow-lite-linux-x64.tar.gz
sudo tar -xzf libtensorflow-lite-linux-x64.tar.gz -C /usr/local

# 方式2: 从源码编译
git clone https://github.com/tensorflow/tensorflow.git
cd tensorflow
./configure
bazel build //tensorflow/lite:libtensorflow-lite.so
sudo cp bazel-bin/tensorflow/lite/libtensorflow-lite.so /usr/local/lib/
sudo mkdir -p /usr/local/include/tensorflow/lite
sudo cp -r tensorflow/lite /usr/local/include/tensorflow/
```

### 2. 检查头文件和库文件路径

确认以下文件存在：
- 头文件: `/usr/local/include/tensorflow/lite/*.h`
- 库文件: `/usr/local/lib/libtensorflow-lite.a` 或 `.so`

如果路径不同，需要修改 `CMakeLists.txt` 中的：
```cmake
set(TFLITE_INCLUDE_DIR "/your/custom/path/include" CACHE PATH "TensorFlow Lite include directory")
set(TFLITE_LIB_DIR "/your/custom/path/lib" CACHE PATH "TensorFlow Lite library directory")
```

## 编译

```bash
cd /home/carlson/wangzimo/realflight_ws
colcon build --packages-select hover_test
source install/setup.bash
```

## 运行

### 方式1: 使用 launch 文件（推荐）

```bash
# 启动神经网络控制节点
ros2 launch hover_test tflite_neural_control.launch.py drone_id:=0

# 指定不同的模型路径
ros2 launch hover_test tflite_neural_control.launch.py \
    drone_id:=0 \
    model_path:=/path/to/your/model.tflite

# 指定不同的配置文件
ros2 launch hover_test tflite_neural_control.launch.py \
    drone_id:=0 \
    config_file:=/path/to/your/config.yaml
```

### 方式2: 直接运行节点

```bash
ros2 run hover_test hover_test_node 0 \
    --ros-args \
    --params-file /home/carlson/wangzimo/realflight_ws/src/hover_test/config/tflite_model.yaml
```

## 配置文件说明

主要配置文件: `config/tflite_model.yaml`

```yaml
/**:
  ros__parameters:
    # 是否启用神经网络控制
    use_neural_control: true
    
    # 模型文件路径（绝对路径）
    model_path: "/path/to/hoverVer1_policy.tflite"
    
    # 控制频率 (Hz) - 应与模型训练时的 dt 匹配
    timer_period: 0.01  # 100 Hz
    
    # 悬停持续时间 (秒)
    hover_duration: 30.0
    
    # 目标点位置 (NED 坐标系)
    target_x: 0.0
    target_y: 0.0
    target_z: 2.0  # 2米高度
    
    # 备用悬停推力（当神经网络失败时使用）
    hover_thrust: 0.71
```

## 工作流程

1. **初始化**: 节点启动时加载 TFLite 模型
2. **等待状态**: 订阅状态机状态，等待进入 `HOVER` 状态
3. **启动控制**: 检测到 `HOVER` 后，发送 `TRAJ` 命令给状态机
4. **执行控制**: 
   - 在 `TRAJ` 状态下，10Hz执行神经网络推理更新动作
   - 100Hz发布体轴角速率和推力控制指令
   - 控制无人机悬停在目标点
5. **结束**: 达到预设时间后，发送 `END_TRAJ` 命令

## 调试

### 查看日志

```bash
ros2 topic echo /rosout
```

关键日志信息：
- `✅ Neural network policy initialized` - 模型加载成功
- `Neural network policy reset` - 进入 TRAJ 状态时重置
- `Neural control | pos=...` - 实时控制状态

### 检查模型加载

如果模型加载失败，检查：
1. 模型文件路径是否正确
2. 文件权限是否可读
3. TFLite 库是否正确安装

### 性能监控

```bash
# 监控控制指令输出
ros2 topic echo /fmu/in/vehicle_rates_setpoint

# 监控当前位置
ros2 topic echo /fmu/out/vehicle_odometry
```

## 常见问题

### Q1: 编译错误 "tensorflow/lite/model.h: No such file or directory"

**A**: TFLite 头文件未找到，检查：
- 是否已安装 TFLite
- CMakeLists.txt 中的 `TFLITE_INCLUDE_DIR` 路径是否正确

### Q2: 链接错误 "undefined reference to tflite::..."

**A**: TFLite 库文件未找到或未正确链接，检查：
- 是否存在 `/usr/local/lib/libtensorflow-lite.a`
- CMakeLists.txt 中的 `TFLITE_LIB_DIR` 路径是否正确

### Q3: 运行时错误 "Failed to load model"

**A**: 模型文件加载失败，检查：
- 配置文件中的 `model_path` 是否为绝对路径
- 文件是否存在且可读
- 模型文件格式是否正确（.tflite）

### Q4: 无人机不响应控制指令

**A**: 可能的原因：
1. 状态机未进入 `TRAJ` 状态 - 检查 `/state/state_drone_0` 话题
2. 传感器数据未就绪 - 确认 odometry、local_position、attitude 话题有数据
3. 神经网络未启用 - 检查 `use_neural_control` 参数

## 文件结构

```
hover_test/
├── config/
│   ├── hoverVer1_policy.tflite       # TFLite 模型文件
│   ├── tflite_model.yaml             # 神经网络控制配置
│   ├── model_info.txt                # 模型详细信息
│   └── tflite_inference_example.cpp  # 推理示例代码
├── include/hover_test/
│   ├── tflite_policy.hpp             # TFLite 推理封装类
│   └── hover_test_node.hpp           # 主节点头文件
├── src/
│   ├── hover_test_node.cpp           # 主节点实现
│   └── hover_test_main.cpp           # 主函数
├── launch/
│   └── tflite_neural_control.launch.py  # 启动文件
└── CMakeLists.txt                    # 构建配置

```

## 下一步

- [ ] 支持动态目标点（实时更新 target_x/y/z）
- [ ] 添加安全检查和故障保护机制
- [ ] 性能优化和实时性分析
- [ ] 支持多无人机协同悬停

## 参考

- TensorFlow Lite C++ API: https://www.tensorflow.org/lite/guide/inference#load_and_run_a_model_in_c
- PX4 Body Rate Control: https://docs.px4.io/main/en/flight_modes/offboard.html
- HoverEnv 训练环境: 见 `config/model_info.txt`

