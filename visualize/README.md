# 飞行数据可视化工具

## 简介

提供两种可视化方式来分析 ROS2 bag 飞行记录：

1. **RViz2 可视化** (visualize_rviz.py) - 实时3D回放，直观展示飞行过程 🎬
2. **Matplotlib 可视化** (visualize.py) - 生成统计图表和详细报告 📊

**重要说明**：Matplotlib 工具只统计和记录 `vehicle_control_mode/flag_control_position_enabled == 0` 时的数据，即仅在位置控制未启用时的飞行数据。

## 安装依赖

### 1. Python 依赖

```bash
pip install -r requirements.txt
```

### 2. ROS2 环境

**重要：** 本工具需要 ROS2 环境来正确解析消息。使用前必须先 source ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/wangzimo/RealFlight/install/setup.bash
```

## 使用方法

### 方法一：RViz2 实时可视化（推荐用于快速查看）

```bash
# 先 source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/wangzimo/RealFlight/install/setup.bash

# 终端1: 启动 RViz2
rviz2 -d visualize/flight_visualization.rviz

# 终端2: 运行可视化脚本
python3 visualize_rviz.py ../fly_log/track_moving_05

# 按 Enter 开始回放
```

详细说明请查看 [README_RVIZ.md](README_RVIZ.md)

### 方法二：Matplotlib 统计图表（推荐用于报告）

```bash
# 先 source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/wangzimo/RealFlight/install/setup.bash

# 方法1: 使用shell脚本（推荐）
./run_visualize.sh ../fly_log/track_moving_05

# 方法2: 直接使用Python
python3 visualize.py ../fly_log/track_moving_05
```

### 批量处理

```bash
for bag in ../fly_log/*/; do
    ./run_visualize.sh "$bag"
done
```

## 两种可视化方式对比

| 特性 | RViz2 | Matplotlib |
|------|-------|-----------|
| 实时回放 | ✅ | ❌ |
| 3D交互 | ✅ 自由旋转 | ⚠️ 有限 |
| 朝向显示 | ✅ 实时箭头 | ✅ 静态箭头 |
| 统计报告 | ❌ | ✅ 详细 |
| 导出图片 | ⚠️ 截图 | ✅ 高质量 |
| 速度控制 | ✅ 可调 | N/A |

**建议**：先用 RViz2 快速查看，再用 Matplotlib 生成报告。

## Matplotlib 输出内容

所有结果保存在 `output/<bag名称>/` 目录下：

### 可视化图表（6张）

1. **01_distance_over_time.png** - 无人机与目标距离随时间变化
2. **02_trajectory_3d.png** - 3D轨迹视图
3. **03_trajectory_topdown.png** - 俯视图（XY平面）
4. **04_angle_over_time.png** - 目标到无人机x轴夹角随时间变化
5. **05_speed_over_time.png** - 无人机和目标速度对比
6. **06_comprehensive_analysis.png** - 综合分析图（多子图）

### 统计报告

**report.md** - 包含以下统计信息：
- 距离统计：平均距离、最大距离、最小距离
- 夹角统计：平均夹角、最大夹角、最小夹角

### 方法二：Matplotlib 统计图表（推荐用于报告）

```bash
# 先 source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/wangzimo/RealFlight/install/setup.bash

# 方法1: 使用shell脚本（推荐）
./run_visualize.sh ../fly_log/track_moving_05

# 方法2: 直接使用Python
python3 visualize.py ../fly_log/track_moving_05
```

### 批量处理

```bash
for bag in ../fly_log/*/; do
    ./run_visualize.sh "$bag"
done
```

## 两种可视化方式对比

| 特性 | RViz2 | Matplotlib |
|------|-------|-----------|
| 实时回放 | ✅ | ❌ |
| 3D交互 | ✅ 自由旋转 | ⚠️ 有限 |
| 朝向显示 | ✅ 实时箭头 | ✅ 静态箭头 |
| 统计报告 | ❌ | ✅ 详细 |
| 导出图片 | ⚠️ 截图 | ✅ 高质量 |
| 速度控制 | ✅ 可调 | N/A |

**建议**：先用 RViz2 快速查看，再用 Matplotlib 生成报告。

## Matplotlib 输出内容

所有结果保存在 `output/<bag名称>/` 目录下：

### 可视化图表（6张）

1. **01_distance_over_time.png** - 无人机与目标距离随时间变化
2. **02_trajectory_3d.png** - 3D轨迹视图
3. **03_trajectory_topdown.png** - 俯视图（XY平面）
4. **04_angle_over_time.png** - 目标到无人机x轴夹角随时间变化
5. **05_speed_over_time.png** - 无人机和目标速度对比
6. **06_comprehensive_analysis.png** - 综合分析图（多子图）

### 统计报告

**report.md** - 包含以下统计信息：
- 距离统计：平均距离、最大距离、最小距离
- 夹角统计：平均夹角、最大夹角、最小夹角
- 速度统计：无人机和目标的速度信息
- 位置信息：起点和终点坐标

## 注意事项
- 速度统计：无人机和目标的速度信息
- 位置信息：起点和终点坐标

## 注意事项

- 确保bag文件包含以下topic：
  - `/fmu/out/vehicle_odometry`
  - `/target/position`
  - `/target/velocity`
  - `/fmu/out/vehicle_control_mode` (用于数据过滤)
- **数据过滤**：只统计 `flag_control_position_enabled == 0` 时的数据
- 使用NED坐标系（North-East-Down）
- 输出目录会自动创建

## 示例

### RViz2 实时可视化

```bash
# 正常速度回放
python3 visualize_rviz.py ../fly_log/track_moving_05

# 2倍速回放
python3 visualize_rviz.py ../fly_log/track_moving_05 --speed 2.0

# 慢速回放（详细分析）
python3 visualize_rviz.py ../fly_log/track_moving_05 --speed 0.5
```

### Matplotlib 统计分析

```bash
# 可视化单个记录
./run_visualize.sh ../fly_log/track_moving_05

# 查看结果
ls output/track_moving_05/
cat output/track_moving_05/report.md
```

## 更多信息

- **RViz2 可视化详细说明**：[README_RVIZ.md](README_RVIZ.md)
- **故障排除和高级用法**：查看各自的 README 文件

