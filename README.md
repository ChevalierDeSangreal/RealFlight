
## 编译说明：

RealFlight现在支持通过CMake参数在实机环境和SITL环境之间切换。

### 编译实机环境 (默认)
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/realflight_ws
colcon build
# 或显式指定
colcon build --cmake-args -DBUILD_ENV=REAL_HARDWARE
```

### 编译SITL环境
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/realflight_ws
colcon build --cmake-args -DBUILD_ENV=SITL
```

### 切换环境时清理缓存
```bash
rm -rf build install
colcon build --cmake-args -DBUILD_ENV=SITL  # 或 REAL_HARDWARE
```

**详细说明请参阅: [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)**

## 运行：
0. 设置ROS DOMAIN
```bash
export ROS_DOMAIN_ID=86
```

1. 启动IsaacSIm仿真+PX4
```bash
ISAACSIM_PYTHON /home/carlson/SimulatorSetup/submodules/PegasusSimulator/examples/1_px4_single_vehicle.py
ISAACSIM_PYTHON /home/carlson/SimulatorSetup/submodules/PegasusSimulator/examples/1_px4_single_raynor.py
```

```bash
source ~/Downloads/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash
source ~/Downloads/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
```

```bash
ISAACSIM_PYTHON /home/wangzimo/Downloads/PegasusSimulator/examples/1_px4_single_raynor.py
ISAACSIM_PYTHON /home/wangzimo/Downloads/PegasusSimulator/examples/1_px4_single_vehicle.py
```

2. 启动DDS
```bash
MicroXRCEAgent udp4 -p 8888
```

3. (state machine)

offboard_state_machine 包支持两种运行模式：
- **onboard 模式（默认）**：用于实机环境，`use_sim_time=False`
- **sitl 模式**：用于仿真环境，`use_sim_time=True`

```bash
cd /home/carlson/wangzimo/realflight_ws
source ./install/setup.bash

# 启动单机状态机（默认 onboard 模式）
ros2 launch offboard_state_machine single_drone_test.launch.py

# 启动 SITL 模式（仿真环境）
ros2 launch offboard_state_machine single_drone_test.launch.py mode:=sitl

# 启动多机状态机（默认 onboard 模式）
ros2 launch offboard_state_machine multi_drone_goto.launch.py

# 启动多机状态机 SITL 模式
ros2 launch offboard_state_machine multi_drone_goto.launch.py mode:=sitl

# 指定无人机 ID 和模式
ros2 launch offboard_state_machine single_drone_test.launch.py drone_id:=0 mode:=sitl
```

### 神经网络控制节点启动

hover_test 包支持两种运行模式：
- **onboard 模式（默认）**：用于实机环境，使用 `model_path`（绝对路径），`use_sim_time=False`
- **sitl 模式**：用于仿真环境，使用 `model_path_sitl`（绝对路径），`use_sim_time=True`

配置文件（`tflite_model.yaml` 和 `tflite_model_50hz.yaml`）中同时包含 `model_path` 和 `model_path_sitl` 两个参数，launch 文件会根据模式自动选择使用哪个路径。两种模式都使用绝对路径，但路径指向不同的环境（实机环境 vs 仿真环境）。

```bash
# 启动神经网络控制节点（默认 onboard 模式）
ros2 launch track_test tflite_neural_control.launch.py
ros2 launch hover_test tflite_neural_control.launch.py
ros2 launch hover_test tflite_neural_control_50hz.launch.py

# 启动 SITL 模式（仿真环境）
ros2 launch hover_test tflite_neural_control.launch.py mode:=sitl
ros2 launch hover_test tflite_neural_control_50hz.launch.py mode:=sitl

# 显式指定 onboard 模式（实机环境，默认）
ros2 launch hover_test tflite_neural_control.launch.py mode:=onboard
ros2 launch hover_test tflite_neural_control_50hz.launch.py mode:=onboard

# 指定无人机 ID 和模式
ros2 launch hover_test tflite_neural_control_50hz.launch.py drone_id:=0 mode:=sitl
```

```bash
ros2 launch track_test tflite_neural_control.launch.py > /home/carlson/wangzimo/tmplog.txt 2>&1
ros2 launch hover_test tflite_neural_control.launch.py > /home/carlson/wangzimo/tmplog.txt 2>&1
```

监视状态变换：
```bash
ros2 topic echo /state/state_drone_0 | awk '/data:/ {
    state = $2; 
    if (state != prev) {
        system("date +%H:%M:%S.%N | tr -d \\47\\n\\47"); 
        printf " State changed: %s -> %s", prev, state;
        if (state == 4) printf " (HOVER - position control!)";
        if (state == 5) printf " (TRAJ - body_rate control)";
        if (state == 6) printf " (END_TRAJ - position control!)";
        printf "\n";
        prev = state;
    }
}'
```

监视控制指令变化：
```bash
ros2 topic echo /fmu/in/offboard_control_mode | awk '
/position:/ {pos = $2}
/body_rate:/ {
    brate = $2;
    mode = pos brate;
    if (mode != prev) {
        system("date +%H:%M:%S.%N | cut -c1-12 | tr -d \"\\n\"");
        if (brate == "true")
            printf " -> BODY_RATE control (TRAJ)\n";
        else if (pos == "true")
            printf " -> POSITION control (Normal)\n";
        prev = mode;
    }
}'
```

监视角速度指令：
```bash
ros2 topic echo /fmu/in/vehicle_rates_setpoint
```

监视控制环：
```bash
ros2 topic echo /fmu/in/offboard_control_mode
```

重新编译：
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/realflight_ws

# 编译单个包（实机环境）
colcon build --packages-select track_test
# 编译单个包（SITL环境）
colcon build --packages-select track_test --cmake-args -DBUILD_ENV=SITL

colcon build --packages-select offboard_state_machine
colcon build --packages-select hover_test

source install/setup.bash
```

```bash
ros2 launch track_test track_test.launch.py drone_id:=0
```


## 环境配置：

Pegasus对应地址：
https://github.com/Temasek-Dynamics/PegasusSimulator.git