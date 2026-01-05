
## 编译说明：

RealFlight现在支持通过CMake参数在实机环境和SITL环境之间切换。

### 编译实机环境 (默认)
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/RealFlight
colcon build
# 或显式指定
colcon build --cmake-args -DBUILD_ENV=REAL_HARDWARE
```

### 编译SITL环境
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/RealFlight
colcon build --cmake-args -DBUILD_ENV=SITL
```

### 切换环境时清理缓存
```bash
rm -rf build install
colcon build --cmake-args -DBUILD_ENV=SITL  # 或 REAL_HARDWARE
```

**详细说明请参阅: [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)**

重新编译：
```bash
source /opt/ros/humble/setup.bash
cd ~/wangzimo/RealFlight

source install/setup.bash

# 编译单个包（实机环境）
colcon build --packages-select track_test
# 编译单个包（SITL环境）
colcon build --packages-select track_test --cmake-args -DBUILD_ENV=SITL

colcon build --packages-select offboard_state_machine
colcon build --packages-select hover_test
# 编译 traj_test 包（实机环境，默认）
colcon build --packages-select traj_test
# 编译 traj_test 包（SITL环境）
colcon build --packages-select traj_test --cmake-args -DBUILD_ENV=SITL


```

```bash
ros2 launch track_test track_test.launch.py drone_id:=0
```

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
cd /home/carlson/wangzimo/RealFlight
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

### 轨迹测试节点启动

traj_test 包用于无人机轨迹跟踪测试，支持圆形轨迹飞行：

#### 快速启动（推荐）

使用 `start_traj.sh` 脚本一键启动状态机和轨迹测试节点（tmux会话）：

```bash
cd ~/wangzimo/RealFlight
source ./install/setup.bash

# 使用默认参数（drone_id=0, mode=onboard）
./scripts/start_traj.sh

# 指定无人机 ID 和模式
./scripts/start_traj.sh 0 onboard    # 实机模式
./scripts/start_traj.sh 0 sitl       # 仿真模式
```

脚本会在 tmux 会话中创建两个窗口：
- **窗口 0 (fsm)**：状态机（offboard_state_machine）
- **窗口 1 (traj_test)**：轨迹测试节点（traj_test）

使用 `Ctrl+B` 然后 `D` 可以退出 tmux 会话（进程在后台继续运行），使用 `tmux attach -t traj_test` 可以重新连接。

#### 手动启动

分别启动状态机和轨迹测试节点：

```bash
# 启动轨迹测试节点（默认配置）
ros2 launch traj_test traj_test.launch.py drone_id:=0

# 指定自定义配置文件
ros2 launch traj_test traj_test.launch.py \
    drone_id:=0 \
    config_file:=/path/to/your/config.yaml
```

**注意**：traj_test 包需要在编译时指定环境（SITL 或 REAL_HARDWARE），运行时不需要 mode 参数。

### 神经网络控制节点启动

#### track_test 包（目标跟踪测试）

track_test 包用于无人机目标跟踪测试，支持两种目标生成模式：
- **静态模式**：在进入TRAJ模式时，在无人机正前方自动生成静止目标
- **话题模式**：通过ROS2话题订阅实时目标位置（用于视觉系统集成）

track_test 包支持两种运行模式：
- **onboard 模式（默认）**：用于实机环境，使用 `model_path`（绝对路径），`use_sim_time=False`
- **sitl 模式**：用于仿真环境，使用 `model_path_sitl`（绝对路径），`use_sim_time=True`

配置文件（`tflite_model.yaml`）中同时包含 `model_path` 和 `model_path_sitl` 两个参数，launch 文件会根据模式自动选择使用哪个路径。

```bash
# 启动 track_test 节点（默认 onboard 模式）
ros2 launch track_test tflite_neural_control.launch.py

# 启动 SITL 模式（仿真环境）
ros2 launch track_test tflite_neural_control.launch.py mode:=sitl

# 显式指定 onboard 模式（实机环境，默认）
ros2 launch track_test tflite_neural_control.launch.py mode:=onboard

# 指定无人机 ID 和模式
ros2 launch track_test tflite_neural_control.launch.py drone_id:=0 mode:=sitl

# 使用简化启动文件（默认 SITL 模式，固定 use_sim_time=True）
ros2 launch track_test track_test.launch.py drone_id:=0

# 指定自定义配置文件
ros2 launch track_test track_test.launch.py \
    drone_id:=0 \
    config_file:=/path/to/your/config.yaml
```

**目标生成模式配置**（在 `config/tflite_model.yaml` 中）：
```yaml
# 静态模式（默认）：在无人机正前方生成目标
use_target_topic: false
target_offset_distance: 1.0  # 目标距离（米）

# 话题模式：订阅ROS2话题获取实时目标
use_target_topic: true
target_position_topic: "/target/position"
target_velocity_topic: "/target/velocity"
```

#### hover_test 包

hover_test 包支持两种运行模式：
- **onboard 模式（默认）**：用于实机环境，使用 `model_path`（绝对路径），`use_sim_time=False`
- **sitl 模式**：用于仿真环境，使用 `model_path_sitl`（绝对路径），`use_sim_time=True`

配置文件（`tflite_model.yaml` 和 `tflite_model_50hz.yaml`）中同时包含 `model_path` 和 `model_path_sitl` 两个参数，launch 文件会根据模式自动选择使用哪个路径。两种模式都使用绝对路径，但路径指向不同的环境（实机环境 vs 仿真环境）。

```bash
# 启动神经网络控制节点（默认 onboard 模式）
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




## 环境配置：

Pegasus对应地址：
https://github.com/Temasek-Dynamics/PegasusSimulator.git