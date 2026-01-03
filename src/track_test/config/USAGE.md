# 快速使用指南

## 1. 编译

```bash
cd ~/wangzimo/RealFlight
colcon build --packages-select track_test
source install/setup.bash
```

## 2. 运行（最简单）

```bash
# 启动神经网络控制
ros2 launch track_test tflite_neural_control.launch.py
```

## 3. 参数说明

在 `config/tflite_model.yaml` 中修改：

- `use_neural_control`: true/false - 是否使用神经网络
- `model_path`: 模型文件的绝对路径
- `target_x/y/z`: 目标位置（米）
- `hover_duration`: 运行时长（秒）
- `timer_period`: 0.01 (100Hz，与模型匹配)

## 4. 观测空间 (18维)

根据 `model_info.txt` 和代码实现：

| 索引 | 含义 | 说明 |
|------|------|------|
| 0-2  | 位置误差 | target_pos - current_pos (NED) |
| 3-5  | 当前速度 | vx, vy, vz (m/s) |
| 6-8  | 目标速度 | target_vx, target_vy, target_vz |
| 9-11 | 姿态角 | roll, pitch, yaw (rad) |
| 12-14| 角速率 | roll_rate, pitch_rate, yaw_rate |
| 15-17| 重力向量 | 机体坐标系下的重力方向 |

## 5. 动作空间 (4维)

神经网络输出：

| 索引 | 含义 | 范围 |
|------|------|------|
| 0 | roll_rate | [-1, 1] rad/s |
| 1 | pitch_rate | [-1, 1] rad/s |
| 2 | yaw_rate | [-1, 1] rad/s |
| 3 | thrust | [-1, 1] → [0, 1] |

## 6. 工作流程图

```
[INIT] → [ARMING] → [TAKEOFF] → [GOTO] → [HOVER]
                                              ↓
                        [track_test 发送 TRAJ 命令]
                                              ↓
                                          [TRAJ 状态]
                                              ↓
                              每 0.01s 执行神经网络推理
                              生成 body rate 控制指令
                                              ↓
                                     持续 hover_duration
                                              ↓
                        [track_test 发送 END_TRAJ 命令]
                                              ↓
                                           [LAND]
```

## 7. 关键订阅话题

- `/state/state_drone_0` - 状态机状态
- `/fmu/out/vehicle_odometry` - 位置信息
- `/fmu/out/vehicle_local_position` - 速度信息
- `/fmu/out/vehicle_attitude` - 姿态信息

## 8. 发布话题

- `/fmu/in/vehicle_rates_setpoint` - 体轴角速率控制指令
- `/state/command_drone_0` - 状态机命令

## 9. 故障排查

**模型不加载？**
- 检查 `model_path` 是否为绝对路径
- 确认文件权限可读

**无人机不响应？**
- 确认状态机进入了 TRAJ 状态
- 检查传感器话题是否有数据

**备用模式？**
- 如果神经网络失败，会自动切换到简单悬停模式（hover_thrust）

