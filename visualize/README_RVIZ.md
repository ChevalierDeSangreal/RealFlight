# RViz2 飞行数据可视化

使用 RViz2 实时回放飞行数据，提供更直观的 3D 可视化效果。

## 功能特点

- 🛩️ **无人机轨迹**：蓝色路径显示无人机飞行轨迹
- 🎯 **目标轨迹**：红色路径显示目标物体运动轨迹
- ➡️ **朝向显示**：箭头实时显示无人机机头方向
- 📏 **距离显示**：实时显示无人机与目标之间的距离
- 🔗 **连接线**：绿色线段连接无人机和目标
- ⏯️ **回放控制**：支持自定义回放速度

## 使用方法

### 1. 基本用法

```bash
cd ~/wangzimo/RealFlight
source install/setup.bash

# 基本使用（1x 速度）
python3 visualize/visualize_rviz.py /path/to/your/bag_file

# 指定回放速度（例如 2x 速度）
python3 visualize/visualize_rviz.py /path/to/your/bag_file --speed 2.0

# 慢速回放（0.5x 速度）
python3 visualize/visualize_rviz.py /path/to/your/bag_file --speed 0.5
```

### 2. 启动 RViz2

在**另一个终端**中启动 RViz2：

```bash
cd ~/wangzimo/RealFlight
source install/setup.bash

# 使用预配置文件
rviz2 -d visualize/flight_visualization.rviz
```

或者手动启动 RViz2 并配置：

```bash
rviz2
```

然后在 RViz2 中添加以下显示项：

1. **Fixed Frame**: 设置为 `map`
2. **Add** → **Path** → Topic: `/visualization/drone_path` (无人机轨迹)
3. **Add** → **Path** → Topic: `/visualization/target_path` (目标轨迹)
4. **Add** → **Pose** → Topic: `/visualization/drone_pose` (无人机位置)
5. **Add** → **Pose** → Topic: `/visualization/target_pose` (目标位置)
6. **Add** → **MarkerArray** → Topic: `/visualization/markers` (标记)

### 3. 完整流程示例

```bash
# 终端1: 启动 RViz2
cd ~/wangzimo/RealFlight
source install/setup.bash
rviz2 -d visualize/flight_visualization.rviz

# 终端2: 运行可视化脚本
cd ~/wangzimo/RealFlight
source install/setup.bash
python3 visualize/visualize_rviz.py rosbag2_track_moving_2025_01_08_XX_XX_XX

# 按 Enter 开始回放
```

## 可视化元素说明

### 轨迹路径
- **蓝色路径**：无人机飞行轨迹
- **红色路径**：目标物体运动轨迹

### 实时标记
- **蓝色箭头**：无人机当前位置和朝向（箭头指向机头方向）
- **红色球体**：目标物体当前位置
- **绿色线段**：无人机与目标之间的连接线
- **白色文字**：无人机与目标之间的实时距离

### 视角操作
- **鼠标左键拖动**：旋转视角
- **鼠标滚轮**：缩放
- **鼠标中键拖动**：平移视角
- **Shift + 左键**：以点为中心旋转

## 参数说明

### `--speed` 回放速度
- `--speed 1.0`：正常速度（默认）
- `--speed 2.0`：2倍速
- `--speed 0.5`：半速
- `--speed 0.1`：慢速（适合详细分析）

## 常见问题

### 1. RViz2 中看不到任何内容

**检查项**：
- 确认 Fixed Frame 设置为 `map`
- 确认已添加正确的显示项
- 检查各显示项的 Topic 名称是否正确
- 确认可视化脚本正在运行

### 2. 轨迹显示不完整

**原因**：可能是 Path 的 Buffer Length 设置太小

**解决**：在 RViz2 中选择 Path 显示项，将 Buffer Length 设置为 1（显示全部历史）

### 3. 标记不显示

**检查**：
- 确认 MarkerArray 的 Topic 为 `/visualization/markers`
- 检查 Namespaces 是否全部勾选（drone, target, connection, distance_text）

### 4. 回放太快或太慢

使用 `--speed` 参数调整回放速度：
```bash
# 慢速回放（适合详细分析）
python3 visualize/visualize_rviz.py your_bag --speed 0.5

# 快速回放（快速浏览）
python3 visualize/visualize_rviz.py your_bag --speed 3.0
```

## 建议工作流程

1. **实时分析**：使用 RViz2 可视化快速查看飞行效果
2. **详细分析**：使用 Matplotlib 生成统计报告和图表
3. **论文/报告**：使用 Matplotlib 生成的图片

```bash
# 步骤1: RViz2 快速查看
python3 visualize/visualize_rviz.py your_bag --speed 2.0

# 步骤2: 生成详细报告
python3 visualize/visualize.py your_bag
```

## 高级用法

### 保存 RViz2 配置

如果你修改了 RViz2 的显示设置，可以保存配置：

1. 在 RViz2 菜单栏：**File** → **Save Config As**
2. 保存为 `my_config.rviz`
3. 下次使用：`rviz2 -d my_config.rviz`

### 录制视频

使用 RViz2 的屏幕录制功能或系统录屏工具录制飞行回放：

```bash
# Linux 下使用 SimpleScreenRecorder 或 OBS Studio
# 启动录屏软件后，运行可视化脚本
```

## 技术细节

### 坐标系
- **Frame**: `map`（固定坐标系）
- **坐标系定义**：NED (North-East-Down)
  - X: 北方
  - Y: 东方
  - Z: 向下（负值表示高度）

### 发布的 Topic
- `/visualization/drone_path` - 无人机轨迹 (nav_msgs/Path)
- `/visualization/target_path` - 目标轨迹 (nav_msgs/Path)
- `/visualization/drone_pose` - 无人机位姿 (geometry_msgs/PoseStamped)
- `/visualization/target_pose` - 目标位姿 (geometry_msgs/PoseStamped)
- `/visualization/markers` - 可视化标记 (visualization_msgs/MarkerArray)

## 故障排除

如果遇到问题，请检查：

```bash
# 检查 ROS2 环境
ros2 topic list

# 检查可视化 topic 是否正在发布
ros2 topic echo /visualization/drone_path

# 检查节点是否运行
ros2 node list
```

---

**提示**：RViz2 可视化和 Matplotlib 可视化各有优势，建议结合使用以获得最佳效果！

