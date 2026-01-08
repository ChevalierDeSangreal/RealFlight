#!/usr/bin/env python3
# coding: utf-8

"""
ROS2 Bag 数据可视化脚本
读取飞行记录并生成可视化图表和统计报告

注意：需要先 source ROS2 环境
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

# ROS2 相关导入
try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
except ImportError as e:
    print("❌ 错误: 无法导入 ROS2 Python 库")
    print("请确保已经 source ROS2 环境:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ./install/setup.bash")
    sys.exit(1)

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def read_ros2_bag(bag_path):
    """读取ROS2 bag文件数据（使用 ROS2 API）
    
    Args:
        bag_path: bag文件路径
        
    Returns:
        dict: 包含各topic数据的字典
    """
    print(f"正在读取 bag 文件: {bag_path}")
    
    # 创建 bag reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 获取 topic 类型信息
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"可用 topic 列表:")
    for topic in sorted(type_map.keys()):
        print(f"  - {topic} ({type_map[topic]})")
    
    # 数据存储
    data = {
        'vehicle_odometry': [],
        'target_position': [],
        'target_velocity': [],
        'vehicle_control_mode': []
    }
    
    # 读取所有消息
    message_count = 0
    while reader.has_next():
        topic, msg_data, timestamp = reader.read_next()
        message_count += 1
        
        timestamp_sec = timestamp / 1e9  # 转换为秒
        
        # 根据 topic 类型反序列化消息
        if topic not in type_map:
            continue
            
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(msg_data, msg_type)
        
        # 根据 topic 名称处理消息
        if '/vehicle_odometry' in topic:
            data['vehicle_odometry'].append({
                'timestamp': timestamp_sec,
                'position': np.array([msg.position[0], msg.position[1], msg.position[2]]),
                'quaternion': np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]]),
                'velocity': np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
            })
            
        elif '/target/position' in topic:
            # PointStamped: msg.point.x, msg.point.y, msg.point.z
            data['target_position'].append({
                'timestamp': timestamp_sec,
                'position': np.array([msg.point.x, msg.point.y, msg.point.z])
            })
            
        elif '/target/velocity' in topic:
            # TwistStamped: msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
            data['target_velocity'].append({
                'timestamp': timestamp_sec,
                'velocity': np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            })
            
        elif '/vehicle_control_mode' in topic:
            data['vehicle_control_mode'].append({
                'timestamp': timestamp_sec,
                'flag_control_position_enabled': msg.flag_control_position_enabled
            })
    
    print(f"\n共找到 {message_count} 条消息")
    print(f"解析完成:")
    print(f"  - vehicle_odometry: {len(data['vehicle_odometry'])} 条")
    print(f"  - target_position: {len(data['target_position'])} 条")
    print(f"  - target_velocity: {len(data['target_velocity'])} 条")
    print(f"  - vehicle_control_mode: {len(data['vehicle_control_mode'])} 条")
    
    return data


# 注意：不再需要手动解析 CDR 格式的函数
# ROS2 API 会自动处理消息的反序列化


def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵
    
    Args:
        q: 四元数 [w, x, y, z]
        
    Returns:
        3x3 旋转矩阵
    """
    w, x, y, z = q
    
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    return R


def compute_angle_between_body_x_and_target(R, quad_pos, target_pos):
    """计算无人机x轴与到目标物体方向的夹角（度数）
    
    Args:
        R: 无人机的旋转矩阵 (3x3)
        quad_pos: 无人机位置 (3,)
        target_pos: 目标位置 (3,)
        
    Returns:
        夹角（度数）
    """
    # 机体x轴在世界坐标系中的方向
    body_x_world = R @ np.array([1.0, 0.0, 0.0])
    
    # 从无人机到目标的方向向量
    direction_to_target = target_pos - quad_pos
    direction_norm = np.linalg.norm(direction_to_target)
    
    if direction_norm < 1e-6:
        return 0.0
    
    direction_to_target_normalized = direction_to_target / direction_norm
    
    # 计算夹角的余弦值
    cos_angle = np.dot(body_x_world, direction_to_target_normalized)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    
    # 转换为角度
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


def process_data(data):
    """处理原始数据，对齐时间戳并计算衍生量
    只保留 flag_control_position_enabled == 0 时的数据
    
    Args:
        data: 原始数据字典
        
    Returns:
        处理后的数据字典
    """
    print("\n正在处理数据...")
    
    # 提取数据
    odom_data = data['vehicle_odometry']
    target_pos_data = data['target_position']
    target_vel_data = data['target_velocity']
    control_mode_data = data['vehicle_control_mode']
    
    if not odom_data or not target_pos_data:
        raise ValueError("数据为空或不完整")
    
    # 检查是否有 control_mode 数据
    if not control_mode_data:
        print("  ⚠️  警告: 未找到 vehicle_control_mode 数据")
        print("  提示: 请确保 bag 文件包含 /fmu/out/vehicle_control_mode topic")
        print("  由于无法确定 flag_control_position_enabled 状态，将跳过所有数据")
        print("  返回空数据集")
        # 返回空的数据结构
        return {
            'times': np.array([]),
            'drone_positions': np.array([]),
            'drone_velocities': np.array([]),
            'drone_rotations': [],
            'target_positions': np.array([]),
            'target_velocities': np.array([]),
            'distances': np.array([]),
            'angles': np.array([])
        }
    
    # 统计信息
    total_points = 0
    filtered_points = 0
    
    # 创建对齐的数据结构
    processed = {
        'times': [],
        'drone_positions': [],
        'drone_velocities': [],
        'drone_rotations': [],
        'target_positions': [],
        'target_velocities': [],
        'distances': [],
        'angles': []
    }
    
    # 以 odometry 数据为基准
    for odom in odom_data:
        t = odom['timestamp']
        total_points += 1
        
        # 检查 flag_control_position_enabled 是否为 0
        control_mode = interpolate_data(control_mode_data, t, 'flag_control_position_enabled')
        if control_mode is None:
            # 如果没有控制模式数据，跳过该点（保守策略）
            continue
        
        # 只处理 flag_control_position_enabled == 0 (False) 的数据
        # 处理bool和int两种类型：0 或 False 都表示未启用
        if control_mode != 0 and control_mode != False:
            continue
        
        filtered_points += 1
        
        # 找到最接近的目标位置
        target_pos = interpolate_data(target_pos_data, t, 'position')
        if target_pos is None:
            continue
            
        # 找到最接近的目标速度
        target_vel = interpolate_data(target_vel_data, t, 'velocity')
        if target_vel is None:
            target_vel = np.zeros(3)
        
        # 计算旋转矩阵
        R = quaternion_to_rotation_matrix(odom['quaternion'])
        
        # 计算距离
        distance = np.linalg.norm(odom['position'] - target_pos)
        
        # 计算夹角
        angle = compute_angle_between_body_x_and_target(R, odom['position'], target_pos)
        
        # 保存数据
        processed['times'].append(t)
        processed['drone_positions'].append(odom['position'])
        processed['drone_velocities'].append(odom['velocity'])
        processed['drone_rotations'].append(R)
        processed['target_positions'].append(target_pos)
        processed['target_velocities'].append(target_vel)
        processed['distances'].append(distance)
        processed['angles'].append(angle)
    
    # 转换为numpy数组
    processed['times'] = np.array(processed['times'])
    processed['drone_positions'] = np.array(processed['drone_positions'])
    processed['drone_velocities'] = np.array(processed['drone_velocities'])
    processed['target_positions'] = np.array(processed['target_positions'])
    processed['target_velocities'] = np.array(processed['target_velocities'])
    processed['distances'] = np.array(processed['distances'])
    processed['angles'] = np.array(processed['angles'])
    
    # 时间归零（从0开始）
    if len(processed['times']) > 0:
        processed['times'] -= processed['times'][0]
    
    print(f"处理完成:")
    print(f"  - 总数据点: {total_points}")
    print(f"  - 过滤后数据点 (flag_control_position_enabled==0): {filtered_points}")
    print(f"  - 最终有效数据点: {len(processed['times'])}")
    
    return processed


def interpolate_data(data_list, target_time, field):
    """在数据列表中插值获取指定时间的值
    
    Args:
        data_list: 数据列表
        target_time: 目标时间
        field: 要提取的字段名
        
    Returns:
        插值后的值
    """
    if not data_list:
        return None
    
    # 找到最接近的两个时间点
    times = [d['timestamp'] for d in data_list]
    
    # 简单的最近邻插值
    idx = np.argmin(np.abs(np.array(times) - target_time))
    
    # 如果时间差太大（>0.1秒），返回None
    if abs(times[idx] - target_time) > 0.1:
        return None
    
    return data_list[idx][field]


def create_visualizations(processed_data, output_dir):
    """创建所有可视化图表
    
    Args:
        processed_data: 处理后的数据
        output_dir: 输出目录
    """
    print("\n正在生成可视化图表...")
    
    times = processed_data['times']
    drone_pos = processed_data['drone_positions']
    drone_vel = processed_data['drone_velocities']
    drone_rotations = processed_data['drone_rotations']
    target_pos = processed_data['target_positions']
    target_vel = processed_data['target_velocities']
    distances = processed_data['distances']
    angles = processed_data['angles']
    
    # 检查数据是否为空
    if len(times) == 0:
        print("  ⚠️  警告: 没有有效数据点，跳过可视化生成")
        print("  提示: 请检查 bag 文件中是否包含 vehicle_control_mode topic")
        print("  或者检查 flag_control_position_enabled 的过滤条件")
        return
    
    # 图1: 距离随时间变化
    plt.figure(figsize=(12, 6))
    plt.plot(times, distances, 'b-', linewidth=2, label='Distance to Target')
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Distance [m]', fontsize=12)
    plt.title('Distance between Drone and Target over Time', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '01_distance_over_time.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成距离图")
    plt.close()
    
    # 图2: 3D轨迹图
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制无人机轨迹
    ax.plot(drone_pos[:, 0], drone_pos[:, 1], drone_pos[:, 2],
            'b-', linewidth=2, label='Drone Trajectory', alpha=0.8)
    
    # 绘制目标轨迹
    ax.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2],
            'r--', linewidth=2, label='Target Trajectory', alpha=0.7)
    
    # 绘制起点
    ax.scatter(drone_pos[0, 0], drone_pos[0, 1], drone_pos[0, 2],
               c='green', s=150, marker='o', label='Drone Start', edgecolors='black', linewidth=2)
    ax.scatter(target_pos[0, 0], target_pos[0, 1], target_pos[0, 2],
               c='orange', s=150, marker='*', label='Target Start', edgecolors='black', linewidth=2)
    
    # 绘制终点
    ax.scatter(drone_pos[-1, 0], drone_pos[-1, 1], drone_pos[-1, 2],
               c='blue', s=150, marker='x', label='Drone End', linewidth=3)
    ax.scatter(target_pos[-1, 0], target_pos[-1, 1], target_pos[-1, 2],
               c='red', s=150, marker='x', label='Target End', linewidth=3)
    
    ax.set_xlabel('X (North) [m]', fontsize=12)
    ax.set_ylabel('Y (East) [m]', fontsize=12)
    ax.set_zlabel('Z (Down) [m]', fontsize=12)
    ax.set_title('3D Trajectory View', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '02_trajectory_3d.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成3D轨迹图")
    plt.close()
    
    # 图3: 俯视图
    plt.figure(figsize=(12, 10))
    plt.plot(drone_pos[:, 0], drone_pos[:, 1], 'b-', linewidth=2, label='Drone Trajectory', alpha=0.8)
    plt.plot(target_pos[:, 0], target_pos[:, 1], 'r--', linewidth=2, label='Target Trajectory', alpha=0.7)
    
    # 每隔一段时间标记无人机朝向
    # 从旋转矩阵中提取机头方向（x轴在XY平面的投影）
    drone_rotations = processed_data['drone_rotations']
    orientation_interval = 0.5  # 每0.5秒标记一次朝向
    arrow_length = 0.3  # 箭头长度（米）
    
    orientation_indices = []
    for i in range(len(times)):
        if i == 0 or times[i] - times[orientation_indices[-1]] >= orientation_interval:
            orientation_indices.append(i)
    
    # 绘制朝向箭头
    for idx in orientation_indices:
        R = drone_rotations[idx]
        # 机头方向在世界坐标系中（与compute_angle函数保持一致的计算方式）
        body_x_world = R @ np.array([1.0, 0.0, 0.0])
        # 只取XY分量（俯视图）
        body_x_xy = body_x_world[:2]
        body_x_xy_norm = np.linalg.norm(body_x_xy)
        if body_x_xy_norm > 1e-6:
            body_x_xy_normalized = body_x_xy / body_x_xy_norm
            # 绘制箭头（箭头方向表示机头朝向）
            plt.arrow(drone_pos[idx, 0], drone_pos[idx, 1],
                     body_x_xy_normalized[0] * arrow_length,
                     body_x_xy_normalized[1] * arrow_length,
                     head_width=0.15, head_length=0.1, fc='cyan', ec='cyan', alpha=0.7, zorder=6)
    
    # 起点
    plt.scatter(drone_pos[0, 0], drone_pos[0, 1], c='green', s=150, marker='o',
                label='Drone Start', edgecolors='black', linewidth=2, zorder=5)
    plt.scatter(target_pos[0, 0], target_pos[0, 1], c='orange', s=150, marker='*',
                label='Target Start', edgecolors='black', linewidth=2, zorder=5)
    
    # 终点
    plt.scatter(drone_pos[-1, 0], drone_pos[-1, 1], c='blue', s=150, marker='x',
                label='Drone End', linewidth=3, zorder=5)
    plt.scatter(target_pos[-1, 0], target_pos[-1, 1], c='red', s=150, marker='x',
                label='Target End', linewidth=3, zorder=5)
    
    plt.xlabel('X (North) [m]', fontsize=12)
    plt.ylabel('Y (East) [m]', fontsize=12)
    plt.title('Top-Down View (XY Plane)', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '03_trajectory_topdown.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成俯视图（包含朝向标记）")
    plt.close()
    
    # 图4: 夹角随时间变化
    plt.figure(figsize=(12, 6))
    plt.plot(times, angles, 'magenta', linewidth=2)
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Angle [degrees]', fontsize=12)
    plt.title('Angle between Drone X-axis and Target Direction over Time', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    plt.axhline(y=90, color='r', linestyle='--', alpha=0.5, label='90° (perpendicular)')
    plt.axhline(y=180, color='r', linestyle='--', alpha=0.5, label='180° (opposite)')
    plt.ylim([0, 180])
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '04_angle_over_time.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成夹角图")
    plt.close()
    
    # 图5: 速度随时间变化（无人机和目标在一个图中）
    drone_speed = np.linalg.norm(drone_vel, axis=1)
    target_speed = np.linalg.norm(target_vel, axis=1)
    
    plt.figure(figsize=(12, 6))
    plt.plot(times, drone_speed, 'b-', linewidth=2, label='Drone Speed', alpha=0.8)
    plt.plot(times, target_speed, 'r--', linewidth=2, label='Target Speed', alpha=0.8)
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Speed [m/s]', fontsize=12)
    plt.title('Drone and Target Speed over Time', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '05_speed_over_time.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成速度图")
    plt.close()
    
    # 图6: 综合分析图（多子图）
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 子图1: XY位置
    axes[0, 0].plot(drone_pos[:, 0], drone_pos[:, 1], 'b-', linewidth=2, label='Drone', alpha=0.8)
    axes[0, 0].plot(target_pos[:, 0], target_pos[:, 1], 'r--', linewidth=2, label='Target', alpha=0.7)
    axes[0, 0].set_xlabel('X [m]', fontsize=10)
    axes[0, 0].set_ylabel('Y [m]', fontsize=10)
    axes[0, 0].set_title('XY Trajectory', fontsize=12, fontweight='bold')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # 子图2: 高度随时间
    axes[0, 1].plot(times, drone_pos[:, 2], 'b-', linewidth=2, label='Drone Z', alpha=0.8)
    axes[0, 1].plot(times, target_pos[:, 2], 'r--', linewidth=2, label='Target Z', alpha=0.7)
    axes[0, 1].set_xlabel('Time [s]', fontsize=10)
    axes[0, 1].set_ylabel('Z (Down) [m]', fontsize=10)
    axes[0, 1].set_title('Height over Time', fontsize=12, fontweight='bold')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 子图3: 距离
    axes[1, 0].plot(times, distances, 'g-', linewidth=2)
    axes[1, 0].set_xlabel('Time [s]', fontsize=10)
    axes[1, 0].set_ylabel('Distance [m]', fontsize=10)
    axes[1, 0].set_title('Distance to Target', fontsize=12, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 子图4: 夹角
    axes[1, 1].plot(times, angles, 'm-', linewidth=2)
    axes[1, 1].set_xlabel('Time [s]', fontsize=10)
    axes[1, 1].set_ylabel('Angle [degrees]', fontsize=10)
    axes[1, 1].set_title('Angle to Target', fontsize=12, fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_ylim([0, 180])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '06_comprehensive_analysis.png'), dpi=150, bbox_inches='tight')
    print("  ✓ 生成综合分析图")
    plt.close()
    
    print("所有图表生成完成！")


def generate_report(processed_data, output_dir, bag_name):
    """生成统计报告（Markdown格式）
    
    Args:
        processed_data: 处理后的数据
        output_dir: 输出目录
        bag_name: bag文件名称
    """
    print("\n正在生成统计报告...")
    
    times = processed_data['times']
    distances = processed_data['distances']
    angles = processed_data['angles']
    drone_vel = processed_data['drone_velocities']
    target_vel = processed_data['target_velocities']
    drone_pos = processed_data['drone_positions']
    target_pos = processed_data['target_positions']
    
    # 检查数据是否为空
    if len(times) == 0:
        print("  ⚠️  警告: 没有有效数据，生成空报告")
        report_path = os.path.join(output_dir, 'report.md')
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(f"# 飞行数据分析报告\n\n")
            f.write(f"**记录名称**: {bag_name}\n\n")
            f.write(f"**生成时间**: {Path(output_dir).name}\n\n")
            f.write("---\n\n")
            f.write("## ⚠️ 数据警告\n\n")
            f.write("**没有有效数据点**\n\n")
            f.write("可能的原因：\n")
            f.write("1. bag 文件中未包含 `/fmu/out/vehicle_control_mode` topic\n")
            f.write("2. 所有数据点的 `flag_control_position_enabled` 都不为 0\n")
            f.write("3. 数据时间戳无法对齐\n\n")
            f.write("请检查 bag 文件内容，确保包含必要的 topic 数据。\n")
        print(f"  ✓ 报告已保存到: {report_path}")
        return
    
    # 计算统计数据
    avg_distance = np.mean(distances)
    max_distance = np.max(distances)
    min_distance = np.min(distances)
    
    avg_angle = np.mean(angles)
    max_angle = np.max(angles)
    min_angle = np.min(angles)
    
    avg_drone_speed = np.mean(np.linalg.norm(drone_vel, axis=1))
    max_drone_speed = np.max(np.linalg.norm(drone_vel, axis=1))
    
    avg_target_speed = np.mean(np.linalg.norm(target_vel, axis=1))
    max_target_speed = np.max(np.linalg.norm(target_vel, axis=1))
    
    total_time = times[-1] if len(times) > 0 else 0
    num_points = len(times)
    
    # 生成Markdown报告
    report_path = os.path.join(output_dir, 'report.md')
    
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(f"# 飞行数据分析报告\n\n")
        f.write(f"**记录名称**: {bag_name}\n\n")
        f.write(f"**生成时间**: {Path(output_dir).name}\n\n")
        f.write("---\n\n")
        
        f.write("## 基本信息\n\n")
        f.write(f"- **总飞行时间**: {total_time:.2f} 秒\n")
        f.write(f"- **数据点数量**: {num_points}\n")
        f.write(f"- **采样频率**: {num_points/total_time:.1f} Hz (平均)\n")
        f.write(f"- **数据过滤**: 仅统计 `flag_control_position_enabled == 0` 时的数据\n\n")
        
        f.write("---\n\n")
        
        f.write("## 距离统计\n\n")
        f.write(f"- **平均距离**: {avg_distance:.3f} m\n")
        f.write(f"- **最大距离**: {max_distance:.3f} m\n")
        f.write(f"- **最小距离**: {min_distance:.3f} m\n\n")
        
        f.write("---\n\n")
        
        f.write("## 夹角统计\n\n")
        f.write(f"- **平均夹角**: {avg_angle:.2f}°\n")
        f.write(f"- **最大夹角**: {max_angle:.2f}°\n")
        f.write(f"- **最小夹角**: {min_angle:.2f}°\n\n")
        
        f.write("---\n\n")
        
        f.write("## 速度统计\n\n")
        f.write("### 无人机速度\n")
        f.write(f"- **平均速度**: {avg_drone_speed:.3f} m/s\n")
        f.write(f"- **最大速度**: {max_drone_speed:.3f} m/s\n\n")
        
        f.write("### 目标物体速度\n")
        f.write(f"- **平均速度**: {avg_target_speed:.3f} m/s\n")
        f.write(f"- **最大速度**: {max_target_speed:.3f} m/s\n\n")
        
        f.write("---\n\n")
        
        f.write("## 位置信息\n\n")
        f.write("### 起点位置\n")
        f.write(f"- **无人机起点**: [{drone_pos[0, 0]:.3f}, {drone_pos[0, 1]:.3f}, {drone_pos[0, 2]:.3f}] m\n")
        f.write(f"- **目标起点**: [{target_pos[0, 0]:.3f}, {target_pos[0, 1]:.3f}, {target_pos[0, 2]:.3f}] m\n\n")
        
        f.write("### 终点位置\n")
        f.write(f"- **无人机终点**: [{drone_pos[-1, 0]:.3f}, {drone_pos[-1, 1]:.3f}, {drone_pos[-1, 2]:.3f}] m\n")
        f.write(f"- **目标终点**: [{target_pos[-1, 0]:.3f}, {target_pos[-1, 1]:.3f}, {target_pos[-1, 2]:.3f}] m\n\n")
        
        f.write("---\n\n")
        
        f.write("## 可视化图表\n\n")
        f.write("1. [距离随时间变化](01_distance_over_time.png)\n")
        f.write("2. [3D轨迹图](02_trajectory_3d.png)\n")
        f.write("3. [俯视图](03_trajectory_topdown.png)\n")
        f.write("4. [夹角随时间变化](04_angle_over_time.png)\n")
        f.write("5. [速度随时间变化](05_speed_over_time.png)\n")
        f.write("6. [综合分析图](06_comprehensive_analysis.png)\n\n")
    
    print(f"  ✓ 报告已保存到: {report_path}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ROS2 Bag 飞行数据可视化工具')
    parser.add_argument('bag_path', type=str, help='ROS2 bag文件路径')
    parser.add_argument('--output-base', type=str, default='visualize/output',
                        help='输出基础目录（默认: visualize/output）')
    
    args = parser.parse_args()
    
    # 获取bag路径和名称
    bag_path = os.path.abspath(args.bag_path)
    bag_name = os.path.basename(bag_path)
    
    if not os.path.exists(bag_path):
        print(f"错误: 找不到bag文件: {bag_path}")
        sys.exit(1)
    
    # 创建输出目录
    output_base = args.output_base
    output_dir = os.path.join(output_base, bag_name)
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"{'='*60}")
    print(f"ROS2 飞行数据可视化工具")
    print(f"{'='*60}")
    print(f"Bag路径: {bag_path}")
    print(f"输出目录: {output_dir}")
    print(f"{'='*60}\n")
    
    try:
        # 读取数据
        raw_data = read_ros2_bag(bag_path)
        
        # 处理数据
        processed_data = process_data(raw_data)
        
        # 生成可视化
        create_visualizations(processed_data, output_dir)
        
        # 生成报告
        generate_report(processed_data, output_dir, bag_name)
        
        print(f"\n{'='*60}")
        print(f"✅ 所有任务完成！")
        print(f"{'='*60}")
        print(f"输出目录: {output_dir}")
        print(f"{'='*60}\n")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

