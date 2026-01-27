# Traj-Track 多机协同脚本

这个文件夹包含用于多机协同的独立启动脚本：
- **drone0**: 运行 `traj_test`，发布目标位置和速度
- **drone3**: 运行 `track_test`，订阅目标位置和速度进行跟踪
- **vicon_to_target**: 运行 `vicon_to_target_node`，将 Vicon 位置数据转换为目标话题

## 文件说明

- `start_drone0_traj.sh` - 启动 drone0 的轨迹测试节点（目标发布者）
- `start_drone3_track.sh` - 启动 drone3 的跟踪测试节点（目标跟踪者）
- `start_vicon_to_target.sh` - 启动 Vicon 到 Target 转换节点（从 Vicon 系统获取目标位置）

## 使用方法

### 1. 启动 drone0（目标发布者）

在 drone0 的机器上运行：

```bash
cd ~/wangzimo/RealFlight
source ./install/setup.bash

# 实机模式（默认）
./scripts/traj_track/start_drone0_traj.sh onboard

# 或仿真模式
./scripts/traj_track/start_drone0_traj.sh sitl
```

**功能**：
- 启动状态机（drone0）
- 启动轨迹测试节点（drone0）
- 发布 `/target/position` 和 `/target/velocity` 话题

### 2. 启动 drone3（目标跟踪者）

在 drone3 的机器上运行：

```bash
cd ~/wangzimo/RealFlight
source ./install/setup.bash

# 实机模式（默认）
./scripts/traj_track/start_drone3_track.sh onboard

# 或仿真模式
./scripts/traj_track/start_drone3_track.sh sitl
```

**功能**：
- 启动状态机（drone3）
- 启动跟踪测试节点（drone3）
- 订阅 `/target/position` 和 `/target/velocity` 话题（从 drone0 接收）

### 3. 启动 Vicon 到 Target 转换节点（可选）

使用 Vicon 系统作为目标位置源时，运行：

```bash
cd ~/wangzimo/RealFlight
source ./install/setup.bash

# 基本使用（默认话题 /vicon/pose，PoseStamped 类型）
./scripts/traj_track/start_vicon_to_target.sh

# 指定 Vicon 话题名称
./scripts/traj_track/start_vicon_to_target.sh /vicon/drone1/pose

# 指定话题名称和模式
./scripts/traj_track/start_vicon_to_target.sh /vicon/drone1/pose onboard

# 完整参数（话题名称、模式、话题类型）
./scripts/traj_track/start_vicon_to_target.sh /vicon/drone1/pose onboard PoseStamped
./scripts/traj_track/start_vicon_to_target.sh /vicon/object1/transform sitl TransformStamped
```

**参数说明**：
- `vicon_topic_name`: Vicon 话题名称（默认：`/vicon/pose`）
- `mode`: 运行模式，`onboard`（默认）或 `sitl`
- `vicon_topic_type`: 话题类型，`PoseStamped`（默认）或 `TransformStamped`

**功能**：
- 订阅 Vicon 位置话题（`PoseStamped` 或 `TransformStamped`）
- 通过位置差分计算速度
- 发布 `/target/position` 和 `/target/velocity` 话题
- 供 `track_test` 节点订阅使用

**使用场景**：
- 使用 Vicon 系统跟踪真实物体作为目标
- 替代 `traj_test` 节点，直接从 Vicon 获取目标位置
- 多机协同时，使用 Vicon 跟踪的目标无人机作为被跟踪目标

## 工作流程

1. **先启动 drone0**：
   - drone0 进入 HOVER 状态
   - 等待 2 秒后进入 TRAJ 状态
   - 开始发布 `/target/position` 和 `/target/velocity`

2. **再启动 drone3**：
   - drone3 进入 HOVER 状态
   - 发布 `/track/ready` 信号（告诉 drone0 已准备好）
   - 等待收到 `/target/position` 数据
   - 收到目标数据后进入 TRAJ 状态
   - 开始使用神经网络控制跟踪目标

## 注意事项

1. **ROS_DOMAIN_ID**：确保所有无人机使用相同的 `ROS_DOMAIN_ID`（脚本中设置为 86）

2. **启动顺序**：
   - 建议先启动 drone0，再启动 drone3
   - drone0 会等待 drone3 发送 `/track/ready` 信号后才进入 TRAJ 状态

3. **网络连接**：
   - 确保所有无人机在同一 ROS 域中
   - 确保可以互相通信

4. **验证连接**：
   ```bash
   # 检查目标话题是否发布
   ros2 topic echo /target/position
   ros2 topic echo /target/velocity
   
   # 检查话题频率
   ros2 topic hz /target/position
   ```

## 故障排查

### drone3 收不到目标数据
- 检查 drone0 是否正在运行
- 检查 `/target/position` 和 `/target/velocity` 话题是否存在
- 检查 ROS_DOMAIN_ID 是否一致

### drone3 不进入 TRAJ 状态
- 确认 `use_target_topic:=true` 已设置（脚本中已包含）
- 检查是否收到 `/target/position` 数据
- 查看日志：`Waiting for target position topic data`

### drone0 不进入 TRAJ 状态
- 检查是否收到 drone3 的 `/track/ready` 信号
- 查看日志：`Waiting for track node to be ready`

### Vicon 转换节点问题
- 检查 Vicon 系统是否正在运行
- 检查 Vicon 话题是否存在：`ros2 topic list | grep vicon`
- 检查话题类型是否正确：`ros2 topic info /vicon/xxx/pose`
- 确认话题名称与脚本参数一致
- 查看节点日志：检查是否收到 Vicon 数据
- 验证输出话题：`ros2 topic echo /target/position`

