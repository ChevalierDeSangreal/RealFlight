# traj_test - 轨迹测试节点

无人机圆形轨迹跟踪测试节点，与 PX4 offboard 状态机集成。

## 编译

### SITL 环境（仿真）
```bash
cd ~/wangzimo/RealFlight
colcon build --packages-select traj_test --cmake-args -DBUILD_ENV=SITL
source install/setup.bash
```

### 实机环境（默认）
```bash
cd ~/wangzimo/RealFlight
colcon build --packages-select traj_test
source install/setup.bash
```

## 运行

### 启动命令

**使用 launch 文件（推荐）：**

**实机模式（默认）：**
```bash
# 使用默认配置
ros2 launch traj_test traj_test.launch.py drone_id:=0 mode:=onboard

# 指定自定义配置文件
ros2 launch traj_test traj_test.launch.py \
    drone_id:=0 \
    mode:=onboard \
    config_file:=/path/to/your/config.yaml
```

**SITL模式（仿真环境）：**
```bash
# 使用默认配置
ros2 launch traj_test traj_test.launch.py drone_id:=0 mode:=sitl

# 指定自定义配置文件
ros2 launch traj_test traj_test.launch.py \
    drone_id:=0 \
    mode:=sitl \
    config_file:=/path/to/your/config.yaml
```

**直接运行节点：**
```bash
# 实机模式
ros2 run traj_test traj_test_node 0 \
    --ros-args \
    --params-file ~/wangzimo/RealFlight/src/traj_test/config/traj_params.yaml \
    --param use_sim_time:=false

# SITL模式
ros2 run traj_test traj_test_node 0 \
    --ros-args \
    --params-file ~/wangzimo/RealFlight/src/traj_test/config/traj_params.yaml \
    --param use_sim_time:=true
```

### 完整使用流程

1. **启动状态机**
   ```bash
   # 实机模式
   ros2 launch offboard_state_machine single_drone_test.launch.py \
       drone_id:=0 mode:=onboard \
       config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_hover.yaml
   
   # SITL模式
   ros2 launch offboard_state_machine single_drone_test.launch.py \
       drone_id:=0 mode:=sitl \
       config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_hover.yaml
   ```

2. **启动轨迹测试节点**
   ```bash
   # 实机模式
   ros2 launch traj_test traj_test.launch.py drone_id:=0 mode:=onboard
   
   # SITL模式
   ros2 launch traj_test traj_test.launch.py drone_id:=0 mode:=sitl
   ```

3. **工作流程**
   - 节点自动检测 HOVER 状态
   - 检测到 HOVER 后自动发送 TRAJ 命令
   - 进入 TRAJ 状态后开始圆形轨迹飞行
   - 完成预设圈数后自动发送 END_TRAJ 命令

## 配置参数

配置文件：`config/traj_params.yaml`

主要参数：
```yaml
/**:
  ros__parameters:
    circle_radius: 2.0          # 圆形轨迹半径 (米)
    circle_center_x: 0.0         # 圆心 X (NED坐标系)
    circle_center_y: 0.0         # 圆心 Y (NED坐标系)
    circle_center_z: -1.2        # 圆心 Z (负值表示高度)
    circle_init_phase: 0.0       # 初始相位 (弧度)
    circle_times: 1              # 飞行圈数
    circle_duration: 20.0        # 单圈持续时间 (秒)
    ramp_up_time: 3.0            # 加速时间 (秒)
    ramp_down_time: 3.0          # 减速时间 (秒)
    timer_period: 0.02           # 控制周期 (50Hz)
    max_speed: -1.0              # 最大速度限制 (米/秒，负值表示不限制)
```

修改配置文件后重新启动节点即可生效，无需重新编译。

## 调试

**监控话题：**
```bash
# 位置设定点
ros2 topic echo /fmu/in/trajectory_setpoint

# 当前位置
ros2 topic echo /fmu/out/vehicle_odometry

# 状态机状态
ros2 topic echo /state/state_drone_0
```

**常见问题：**
- 无人机不动：检查状态机是否正常运行，是否已进入 HOVER 状态
- 轨迹不平滑：增加 `ramp_up_time` 和 `ramp_down_time`，或减小 `max_speed`
