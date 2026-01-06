# track_test - 目标追踪测试包

无人机目标追踪测试包，包含基于神经网络策略的追踪控制节点和目标物体轨迹发布节点。

## 编译

```bash
cd ~/wangzimo/RealFlight

# SITL 环境（仿真）
colcon build --packages-select track_test --cmake-args -DBUILD_ENV=SITL

# 实机环境（默认）
colcon build --packages-select track_test

source install/setup.bash
```

**注意**: 需要先安装 TensorFlow Lite，详见 [README_TFLITE.md](README_TFLITE.md)

---

## 使用方法

### 1. 发布圆周运动目标 (target_publisher_node)

这个节点会发布一个做圆周运动的目标物体位置和速度。

**基本使用**:

```bash
# 使用默认配置（实机模式，使用系统时钟）
ros2 launch track_test target_publisher.launch.py use_sim_time:=false

# SITL 模式（使用ROS模拟时钟）
ros2 launch track_test target_publisher.launch.py use_sim_time:=true

# 使用自定义配置
ros2 launch track_test target_publisher.launch.py \
    use_sim_time:=false \
    config_file:=/path/to/your/config.yaml
```

**重要**: `use_sim_time` 参数决定节点使用的时钟源：
- `use_sim_time:=true` (SITL模式): 使用ROS模拟时钟，与仿真环境同步
- `use_sim_time:=false` (实机模式): 使用系统时钟，避免模拟时间未推进的问题

**配置参数** (`config/target_publisher_params.yaml`):

```yaml
/**:
  ros__parameters:
    circle_radius: 2.0          # 圆形轨迹半径 [m]
    circle_center_x: 0.0        # 圆心 X 坐标 (NED) [m]
    circle_center_y: 0.0        # 圆心 Y 坐标 (NED) [m]
    circle_center_z: -1.2       # 圆心 Z 坐标 (NED，负值=高度) [m]
    circle_init_phase: 0.0      # 初始相位角 [rad]
    circle_times: 3             # 圆周运动圈数
    circle_duration: 20.0       # 单圈持续时间 [s]
    ramp_up_time: 3.0           # 加速时间 [s]
    ramp_down_time: 3.0         # 减速时间 [s]
    stationary_time: 3.0        # 初始静止时间 [s]
    timer_period: 0.02          # 发布周期 (50Hz) [s]
    max_speed: -1.0             # 最大线速度 [m/s] (负值=不限制)
    use_sim_time: false         # 是否使用模拟时间 (true=SITL, false=实机)
```

**发布的话题**:
- `/target/position` (geometry_msgs/PointStamped) - 目标位置
- `/target/velocity` (geometry_msgs/TwistStamped) - 目标速度

---

### 2. 无人机追踪控制 (track_test_node)

使用神经网络策略控制无人机追踪移动目标。

**完整使用流程**:

#### 步骤 1: 启动状态机

```bash
# 实机模式
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 mode:=onboard \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_track.yaml

# SITL 模式
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 mode:=sitl \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_track.yaml
```

#### 步骤 2: 启动追踪控制节点

```bash
# 实机模式 - 话题模式（订阅目标发布节点）
ros2 launch track_test track_test.launch.py \
    drone_id:=0 \
    mode:=onboard \
    use_target_topic:=true

# SITL 模式
ros2 launch track_test track_test.launch.py \
    drone_id:=0 \
    mode:=sitl \
    use_target_topic:=true

# 静态模式（无人机前方固定位置的静止目标）
ros2 launch track_test track_test.launch.py \
    drone_id:=0 \
    mode:=onboard \
    use_target_topic:=false
```

#### 步骤 3: 启动目标发布节点

```bash
# 实机模式（使用系统时钟）
ros2 launch track_test target_publisher.launch.py use_sim_time:=false

# SITL 模式（使用ROS模拟时钟）
ros2 launch track_test target_publisher.launch.py use_sim_time:=true

# 使用自定义配置文件
ros2 launch track_test target_publisher.launch.py \
    use_sim_time:=false \
    config_file:=/path/to/your/target_publisher_params.yaml
```

**注意**: 
- 在 SITL 模式下，**必须**设置 `use_sim_time:=true`，否则轨迹时间计算会出错
- 在实机模式下，**必须**设置 `use_sim_time:=false`，使用系统时钟
- 节点启动时会显示使用的时钟模式：`Clock mode: SIM_TIME` 或 `Clock mode: SYSTEM_TIME`



---

## 监控和调试

### 查看话题

```bash
# 目标位置
ros2 topic echo /target/position

# 目标速度
ros2 topic echo /target/velocity

# 无人机轨迹设定点
ros2 topic echo /fmu/in/trajectory_setpoint

# 无人机当前位置
ros2 topic echo /fmu/out/vehicle_odometry

# 状态机状态
ros2 topic echo /state/state_drone_0

# 检查发布频率
ros2 topic hz /target/position
```

### 常见问题

**1. 目标位置不发布**
```bash
# 检查节点是否运行
ros2 node list | grep target_publisher

# 检查话题
ros2 topic list | grep target
```

**2. 无人机不追踪目标**
- 确认状态机已进入 TRACK 状态
- 确认 `use_target_topic:=true` 参数已设置
- 检查 TensorFlow Lite 模型文件路径是否正确

**3. 轨迹不平滑**
- 增加 `ramp_up_time` 和 `ramp_down_time`
- 减小 `max_speed` 参数

**4. 目标位置不更新或速度一直为0**
- 检查 `use_sim_time` 参数是否正确设置：
  - SITL模式必须使用 `use_sim_time:=true`
  - 实机模式必须使用 `use_sim_time:=false`
- 查看节点日志，确认时钟模式是否正确：
  - 应显示 `Clock mode: SIM_TIME (ROS clock)` 或 `Clock mode: SYSTEM_TIME (steady_clock)`
- 如果使用模拟时间，确保 `/clock` 话题正在发布（SITL环境）

---

## 目录结构

```
track_test/
├── config/
│   ├── target_publisher_params.yaml  # 目标发布节点配置
│   ├── tflite_model.yaml             # 追踪控制节点配置
│   └── *.tflite                      # 神经网络模型文件
├── include/track_test/
│   ├── target_publisher_node.hpp     # 目标发布节点
│   ├── target_generator.hpp          # 目标生成器
│   └── tflite_policy.hpp             # TFLite 策略接口
├── launch/
│   ├── target_publisher.launch.py    # 启动目标发布节点
│   └── track_test.launch.py          # 启动追踪控制节点
├── scripts/
│   └── publish_target.py             # 手动发布目标位置脚本
└── src/
    ├── target_publisher_node.cpp     # 目标发布节点实现
    ├── target_generator.cpp          # 目标生成器实现
    └── track_test_node.cpp           # 追踪控制节点实现
```

---

## 参考资料

- [README_TFLITE.md](README_TFLITE.md) - TensorFlow Lite 安装说明
- [PX4 Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)
