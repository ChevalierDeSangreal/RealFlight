#!/usr/bin/env python3
# coding: utf-8

"""
ROS2 Bag 数据 RViz2 可视化脚本
读取飞行记录并在 RViz2 中实时回放

注意：需要先 source ROS2 环境
"""

import os
import sys
import argparse
import numpy as np
import time
from pathlib import Path

# ROS2 相关导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Header, ColorRGBA
    from builtin_interfaces.msg import Duration
    from tf2_ros import StaticTransformBroadcaster
except ImportError as e:
    print("❌ 错误: 无法导入 ROS2 Python 库")
    print("请确保已经 source ROS2 环境:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ./install/setup.bash")
    sys.exit(1)


class FlightVisualizer(Node):
    """飞行数据可视化节点"""
    
    def __init__(self, bag_path, playback_speed=1.0):
        super().__init__('flight_visualizer')
        
        self.bag_path = bag_path
        self.playback_speed = playback_speed
        
        # 创建发布器
        self.drone_path_pub = self.create_publisher(Path, '/visualization/drone_path', 10)
        self.target_path_pub = self.create_publisher(Path, '/visualization/target_path', 10)
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/visualization/drone_pose', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/visualization/target_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/markers', 10)
        
        # 创建静态 TF 发布器（用于发布 map 坐标系）
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 数据存储
        self.drone_path_msg = Path()
        self.drone_path_msg.header.frame_id = "odom"  # 使用 odom 作为固定坐标系
        
        self.target_path_msg = Path()
        self.target_path_msg.header.frame_id = "odom"
        
        # 发布静态 TF（odom -> map，单位变换）
        self.publish_static_tf()
        
        # 等待发布器就绪
        import time
        time.sleep(0.5)
        
        self.get_logger().info("✅ 节点初始化完成，发布器已就绪")
        
        self.data = {
            'vehicle_odometry': [],
            'target_position': [],
            'target_velocity': []
        }
        
        self.get_logger().info(f"正在读取 bag 文件: {bag_path}")
        self.load_bag_data()
        
        self.get_logger().info(f"数据加载完成，共 {len(self.data['vehicle_odometry'])} 个无人机数据点")
        self.get_logger().info(f"开始以 {playback_speed}x 速度回放...")
        
    def load_bag_data(self):
        """加载 bag 文件数据"""
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # 获取 topic 类型信息
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        # 读取所有消息
        while reader.has_next():
            topic, msg_data, timestamp = reader.read_next()
            timestamp_sec = timestamp / 1e9
            
            if topic not in type_map:
                continue
                
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(msg_data, msg_type)
            
            # 根据 topic 名称处理消息
            if '/vehicle_odometry' in topic:
                self.data['vehicle_odometry'].append({
                    'timestamp': timestamp_sec,
                    'position': np.array([msg.position[0], msg.position[1], msg.position[2]]),
                    'quaternion': np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]]),
                    'velocity': np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
                })
                
            elif '/target/position' in topic:
                self.data['target_position'].append({
                    'timestamp': timestamp_sec,
                    'position': np.array([msg.point.x, msg.point.y, msg.point.z])
                })
                
            elif '/target/velocity' in topic:
                self.data['target_velocity'].append({
                    'timestamp': timestamp_sec,
                    'velocity': np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
                })
    
    def publish_static_tf(self):
        """发布静态 TF 变换（odom -> map，单位变换）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "map"
        
        # 单位变换（不旋转，不平移，map 和 odom 重合）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("✅ 已发布静态 TF: odom -> map")
    
    def interpolate_target_position(self, timestamp):
        """插值获取目标位置"""
        if not self.data['target_position']:
            return None
        
        times = [d['timestamp'] for d in self.data['target_position']]
        idx = np.argmin(np.abs(np.array(times) - timestamp))
        
        if abs(times[idx] - timestamp) > 0.1:
            return None
        
        return self.data['target_position'][idx]['position']
    
    def create_drone_marker(self, position, quaternion, idx):
        """创建无人机标记（箭头表示朝向）"""
        marker = Marker()
        marker.header.frame_id = "odom"  # 使用 odom 坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone"
        marker.id = idx
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        
        # 姿态（四元数）
        marker.pose.orientation.w = float(quaternion[0])
        marker.pose.orientation.x = float(quaternion[1])
        marker.pose.orientation.y = float(quaternion[2])
        marker.pose.orientation.z = float(quaternion[3])
        
        # 大小
        marker.scale.x = 0.5  # 箭头长度
        marker.scale.y = 0.1  # 箭头宽度
        marker.scale.z = 0.1  # 箭头高度
        
        # 颜色（蓝色）
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def create_target_marker(self, position, idx):
        """创建目标标记（球体）"""
        marker = Marker()
        marker.header.frame_id = "odom"  # 使用 odom 坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        
        # 大小
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 颜色（红色）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker
    
    def create_connection_line(self, drone_pos, target_pos, idx):
        """创建无人机到目标的连接线"""
        marker = Marker()
        marker.header.frame_id = "odom"  # 使用 odom 坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "connection"
        marker.id = idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 添加两个点
        p1 = Point()
        p1.x = float(drone_pos[0])
        p1.y = float(drone_pos[1])
        p1.z = float(drone_pos[2])
        marker.points.append(p1)
        
        p2 = Point()
        p2.x = float(target_pos[0])
        p2.y = float(target_pos[1])
        p2.z = float(target_pos[2])
        marker.points.append(p2)
        
        # 线宽
        marker.scale.x = 0.02
        
        # 颜色（绿色）
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        return marker
    
    def create_distance_text(self, position, distance, idx):
        """创建距离文本标记"""
        marker = Marker()
        marker.header.frame_id = "odom"  # 使用 odom 坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "distance_text"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 位置（在无人机上方）
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) - 0.3
        
        # 文本内容
        marker.text = f"{distance:.2f}m"
        
        # 大小
        marker.scale.z = 0.2
        
        # 颜色（白色）
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def playback(self):
        """回放飞行数据"""
        if not self.data['vehicle_odometry']:
            self.get_logger().error("没有数据可以回放")
            return
        
        # 归零时间戳
        start_time = self.data['vehicle_odometry'][0]['timestamp']
        
        playback_start = time.time()
        
        for i, odom in enumerate(self.data['vehicle_odometry']):
            if not rclpy.ok():
                break
            
            # 计算应该等待的时间
            data_time = odom['timestamp'] - start_time
            elapsed_time = (time.time() - playback_start) * self.playback_speed
            wait_time = data_time - elapsed_time
            
            if wait_time > 0:
                time.sleep(wait_time / self.playback_speed)
            
            # 添加到轨迹
            drone_pose = PoseStamped()
            drone_pose.header.frame_id = "odom"  # 使用 odom 坐标系
            drone_pose.header.stamp = self.get_clock().now().to_msg()
            drone_pose.pose.position.x = float(odom['position'][0])
            drone_pose.pose.position.y = float(odom['position'][1])
            drone_pose.pose.position.z = float(odom['position'][2])
            drone_pose.pose.orientation.w = float(odom['quaternion'][0])
            drone_pose.pose.orientation.x = float(odom['quaternion'][1])
            drone_pose.pose.orientation.y = float(odom['quaternion'][2])
            drone_pose.pose.orientation.z = float(odom['quaternion'][3])
            
            self.drone_path_msg.poses.append(drone_pose)
            self.drone_path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 获取目标位置
            target_pos = self.interpolate_target_position(odom['timestamp'])
            
            if target_pos is not None:
                target_pose = PoseStamped()
                target_pose.header.frame_id = "odom"  # 使用 odom 坐标系
                target_pose.header.stamp = self.get_clock().now().to_msg()
                target_pose.pose.position.x = float(target_pos[0])
                target_pose.pose.position.y = float(target_pos[1])
                target_pose.pose.position.z = float(target_pos[2])
                target_pose.pose.orientation.w = 1.0
                
                self.target_path_msg.poses.append(target_pose)
                self.target_path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 发布轨迹
            self.drone_path_pub.publish(self.drone_path_msg)
            if target_pos is not None:
                self.target_path_pub.publish(self.target_path_msg)
            
            # 发布当前位置
            self.drone_pose_pub.publish(drone_pose)
            if target_pos is not None:
                self.target_pose_pub.publish(target_pose)
            
            # 创建标记
            markers = MarkerArray()
            
            # 无人机标记
            drone_marker = self.create_drone_marker(
                odom['position'], odom['quaternion'], 0)
            markers.markers.append(drone_marker)
            
            # 目标标记
            if target_pos is not None:
                target_marker = self.create_target_marker(target_pos, 1)
                markers.markers.append(target_marker)
                
                # 连接线
                connection = self.create_connection_line(
                    odom['position'], target_pos, 2)
                markers.markers.append(connection)
                
                # 距离文本
                distance = np.linalg.norm(odom['position'] - target_pos)
                distance_text = self.create_distance_text(
                    odom['position'], distance, 3)
                markers.markers.append(distance_text)
            
            # 发布标记
            self.marker_pub.publish(markers)
            
            # 每 10 个数据点打印一次进度
            if i % 10 == 0:
                progress = (i + 1) / len(self.data['vehicle_odometry']) * 100
                self.get_logger().info(
                    f"回放进度: {progress:.1f}% ({i+1}/{len(self.data['vehicle_odometry'])}) | "
                    f"位置: [{odom['position'][0]:.2f}, {odom['position'][1]:.2f}, {odom['position'][2]:.2f}]")
            
            # 前几个点打印调试信息
            if i < 3:
                self.get_logger().info(
                    f"发布数据点 {i+1}: 无人机位置 [{odom['position'][0]:.2f}, {odom['position'][1]:.2f}, {odom['position'][2]:.2f}]")
        
        self.get_logger().info("✅ 回放完成！")
        self.get_logger().info("RViz2 将继续显示最终状态，按 Ctrl+C 退出")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ROS2 Bag 飞行数据 RViz2 可视化工具')
    parser.add_argument('bag_path', type=str, help='ROS2 bag文件路径')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度倍率（默认: 1.0）')
    
    args = parser.parse_args()
    
    # 获取bag路径
    bag_path = os.path.abspath(args.bag_path)
    
    if not os.path.exists(bag_path):
        print(f"❌ 错误: 找不到bag文件: {bag_path}")
        sys.exit(1)
    
    print(f"{'='*60}")
    print(f"ROS2 飞行数据 RViz2 可视化工具")
    print(f"{'='*60}")
    print(f"Bag路径: {bag_path}")
    print(f"回放速度: {args.speed}x")
    print(f"{'='*60}\n")
    
    # 初始化 ROS2
    rclpy.init()
    
    try:
        # 创建可视化节点
        visualizer = FlightVisualizer(bag_path, args.speed)
        
        print("\n请在另一个终端运行:")
        print("  rviz2 -d $(ros2 pkg prefix --share offboard_state_machine)/rviz/flight_visualization.rviz")
        print("或者手动启动 RViz2 并添加以下显示:")
        print("  - Path: /visualization/drone_path (蓝色)")
        print("  - Path: /visualization/target_path (红色)")
        print("  - PoseStamped: /visualization/drone_pose")
        print("  - PoseStamped: /visualization/target_pose")
        print("  - MarkerArray: /visualization/markers")
        print(f"\n按 Enter 开始回放...")
        input()
        
        # 开始回放
        visualizer.playback()
        
        # 保持运行直到用户中断
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

