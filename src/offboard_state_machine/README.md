# offboard_state_machine - PX4 Offboard 状态机

PX4 offboard控制状态机，用于管理无人机的起飞、悬停、轨迹执行和降落。

## 功能概述

- ✅ 自动完成起飞、悬停、降落流程
- ✅ **支持两种TRAJ控制模式**：Rate Control 和 Position Control
- ✅ 与外部控制节点协同工作（`hover_test`, `traj_test`）

---

## ⭐ TRAJ状态控制模式

### 核心参数：`traj_use_position_control`

| 参数值 | 控制模式 | 外部节点 | 发布话题 |
|--------|----------|----------|----------|
| **false** | **Rate Control** | `hover_test` | `/fmu/in/vehicle_rates_setpoint` |
| **true** | **Position Control** | `traj_test` | `/fmu/in/trajectory_setpoint` |

### 工作流程

**完整状态转换流程：**
```
INIT → ARMING → TAKEOFF → [可选] GOTO → HOVER → TRAJ → END_TRAJ → LAND → DONE
```

**Rate Control模式（hover_test）：**
```
TAKEOFF → [可选] GOTO → HOVER → TRAJ(rate) → hover_test发布角速率+推力 → END_TRAJ → LAND
```

**Position Control模式（traj_test）：**
```
TAKEOFF → [可选] GOTO → HOVER → TRAJ(position) → traj_test发布位置+速度 → END_TRAJ → LAND
```

**说明：**
- `GOTO` 状态是可选的，如果配置了 `goto_x/y/z` 参数（不为NaN），则在 `TAKEOFF` 完成后会自动进入 `GOTO` 状态
- 如果没有配置 `goto_x/y/z` 或参数为NaN，则 `TAKEOFF` 完成后直接进入 `HOVER` 状态
- `TRAJ` 状态通过外部命令（`/state/command_drone_X`）触发
- `END_TRAJ` 状态通过外部命令触发，等待 `end_traj_wait_time` 秒后自动进入 `LAND` 状态

---

## 状态机状态说明

状态机包含以下状态（按执行顺序）：

| 状态 | 值 | 说明 |
|------|---|------|
| `INIT` | 0 | 初始化状态，等待系统就绪 |
| `ARMING` | 1 | 上锁状态，等待进入offboard模式并解锁 |
| `TAKEOFF` | 2 | 起飞状态，自动起飞到指定高度 |
| `GOTO` | 3 | 目标点飞行状态（可选），如果配置了 `goto_x/y/z` 参数 |
| `HOVER` | 4 | 悬停状态，等待外部命令进入TRAJ状态 |
| `TRAJ` | 5 | 轨迹执行状态，由外部节点控制（`traj_test` 或 `hover_test`） |
| `END_TRAJ` | 6 | 轨迹结束状态，等待指定时间后自动降落 |
| `LAND` | 7 | 降落状态，自动降落到地面 |
| `DONE` | 8 | 完成状态，任务结束 |

---

## 快速开始

### 编译

```bash
cd ~/wangzimo/RealFlight
colcon build --packages-select offboard_state_machine
source install/setup.bash
```

### 使用示例

#### 1. Hover任务（Rate Control）

**实机模式（默认）：**
```bash
# 启动状态机（onboard模式，默认）
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 \
    mode:=onboard \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_hover.yaml

# 启动hover_test节点
ros2 launch hover_test tflite_neural_control.launch.py drone_id:=0 mode:=onboard
```

**SITL模式（仿真环境）：**
```bash
# 启动状态机（sitl模式）
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 \
    mode:=sitl \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_hover.yaml

# 启动hover_test节点（sitl模式）
ros2 launch hover_test tflite_neural_control.launch.py drone_id:=0 mode:=sitl
```

#### 2. Traj任务（Position Control）

**实机模式（默认）：**
```bash
# 启动状态机（onboard模式，默认）
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 \
    mode:=onboard \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_traj.yaml

# 启动traj_test节点
ros2 launch traj_test traj_test.launch.py drone_id:=0
```

**SITL模式（仿真环境）：**
```bash
# 启动状态机（sitl模式）
ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=0 \
    mode:=sitl \
    config_file:=~/wangzimo/RealFlight/src/offboard_state_machine/config/fsm_traj.yaml

# 启动traj_test节点
ros2 launch traj_test traj_test.launch.py drone_id:=0
```

**注意：**
- `mode:=onboard` 用于实机环境，`use_sim_time=False`（默认）
- `mode:=sitl` 用于仿真环境，`use_sim_time=True`
- 如果不指定 `mode` 参数，默认为 `onboard` 模式

#### 3. 在launch文件中直接设置

```python
fsm_node = Node(
    package="offboard_state_machine",
    executable="offboard_fsm_node",
    parameters=[{
        "drone_id": 0,
        "takeoff_alt": 1.2,
        "goto_z": -1.2,
        
        # 关键参数：设置TRAJ控制模式
        "traj_use_position_control": False,  # Rate Control (hover_test)
        # "traj_use_position_control": True,  # Position Control (traj_test)
        
        "end_traj_wait_time": 5.0,
    }],
)
```

---

## 运行模式说明

offboard_state_machine 包支持两种运行模式：

| 模式 | 参数值 | 说明 | use_sim_time |
|------|--------|------|--------------|
| **onboard** | `mode:=onboard` | 实机环境（默认） | `False` |
| **sitl** | `mode:=sitl` | 仿真环境 | `True` |

**使用方式：**
- 通过 launch 文件的 `mode` 参数控制
- `mode` 参数会自动设置节点的 `use_sim_time` 参数
- 配置文件（YAML）中不包含 SITL 模式设置，由 launch 文件统一管理

---

## 关键配置参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| **`traj_use_position_control`** | bool | **false** | **false=Rate Control, true=Position Control** |
| `drone_id` | int | 0 | 无人机ID |
| `takeoff_alt` | double | 1.2 | 起飞高度（米） |
| `goto_x` | double | NaN | GOTO目标X坐标（NED坐标系，米） |
| `goto_y` | double | NaN | GOTO目标Y坐标（NED坐标系，米） |
| `goto_z` | double | NaN | GOTO目标Z坐标（NED，负值表示高度） |
| `goto_tol` | double | 0.1 | GOTO到达容差（米） |
| `goto_max_vel` | double | 0.8 | GOTO最大速度（米/秒） |
| `goto_duration` | double | 2.0 | GOTO最小持续时间（秒） |
| `end_traj_wait_time` | double | 5.0 | END_TRAJ状态悬停时间（秒） |
| `landing_time` | double | 5.0 | 降落时间（秒） |

完整参数列表请参考配置文件：
- `config/fsm_traj.yaml` - Rate Control配置（TRAJ模式）
- `config/fsm_hover.yaml` - Position Control配置（HOVER模式）

---

## 话题接口

### 订阅
- `/fmu/out/vehicle_status_v1` - PX4状态
- `/fmu/out/vehicle_odometry` - 里程计数据
- `/state/command_drone_X` - 状态切换命令（外部节点发送，如 `traj_test`, `hover_test`）

### 发布
- `/fmu/in/offboard_control_mode` - Offboard控制模式
- `/fmu/in/trajectory_setpoint` - 轨迹setpoint（TAKEOFF、GOTO、HOVER、END_TRAJ、LAND状态）
- `/state/state_drone_X` - 当前状态（INT32，状态值：0=INIT, 1=ARMING, 2=TAKEOFF, 3=GOTO, 4=HOVER, 5=TRAJ, 6=END_TRAJ, 7=LAND, 8=DONE）

---

## 参考

- PX4 Offboard Control: https://docs.px4.io/main/en/flight_modes/offboard.html
- 配置文件示例：`config/fsm_traj.yaml`, `config/fsm_hover.yaml`
