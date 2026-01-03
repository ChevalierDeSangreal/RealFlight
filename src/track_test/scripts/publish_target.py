#!/usr/bin/env python3
"""
示例：发布目标位置话题

这个脚本演示如何发布目标位置话题，用于测试track_test的话题模式。

使用方法:
1. 在config/tflite_model.yaml中设置 use_target_topic: true
2. 启动track_test节点
3. 运行此脚本发布目标位置

示例:
    python3 publish_target.py --x 2.0 --y 1.0 --z -1.5 --rate 10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TwistStamped
import argparse
import math


class TargetPublisher(Node):
    def __init__(self, target_x, target_y, target_z, rate_hz, mode='static'):
        super().__init__('target_publisher')
        
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.rate_hz = rate_hz
        self.mode = mode
        
        # Publishers
        self.position_pub = self.create_publisher(
            PointStamped, 
            '/target/position', 
            10)
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/target/velocity',
            10)
        
        # Timer
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # For circular motion
        self.start_time = self.get_clock().now()
        
        self.get_logger().info(f'Target Publisher Started')
        self.get_logger().info(f'  Mode: {mode}')
        self.get_logger().info(f'  Target: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]')
        self.get_logger().info(f'  Rate: {rate_hz} Hz')
    
    def timer_callback(self):
        # Position message
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        
        # Velocity message
        vel_msg = TwistStamped()
        vel_msg.header.stamp = pos_msg.header.stamp
        vel_msg.header.frame_id = 'map'
        
        if self.mode == 'static':
            # Static target
            pos_msg.point.x = self.target_x
            pos_msg.point.y = self.target_y
            pos_msg.point.z = self.target_z
            
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            
        elif self.mode == 'circle':
            # Circular motion
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            radius = 1.0
            angular_vel = 0.5  # rad/s
            
            theta = angular_vel * elapsed
            pos_msg.point.x = self.target_x + radius * math.cos(theta)
            pos_msg.point.y = self.target_y + radius * math.sin(theta)
            pos_msg.point.z = self.target_z
            
            vel_msg.twist.linear.x = -radius * angular_vel * math.sin(theta)
            vel_msg.twist.linear.y = radius * angular_vel * math.cos(theta)
            vel_msg.twist.linear.z = 0.0
            
        elif self.mode == 'line':
            # Linear motion
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            velocity = 0.2  # m/s
            
            pos_msg.point.x = self.target_x + velocity * elapsed
            pos_msg.point.y = self.target_y
            pos_msg.point.z = self.target_z
            
            vel_msg.twist.linear.x = velocity
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
        
        # Publish
        self.position_pub.publish(pos_msg)
        self.velocity_pub.publish(vel_msg)
        
        # Log (throttled)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % (self.rate_hz * 2) == 0:  # Log every 2 seconds
            self.get_logger().info(
                f'Publishing target: [{pos_msg.point.x:.2f}, '
                f'{pos_msg.point.y:.2f}, {pos_msg.point.z:.2f}]')


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish target position for track_test')
    parser.add_argument('--x', type=float, default=2.0, help='Target X position [m]')
    parser.add_argument('--y', type=float, default=0.0, help='Target Y position [m]')
    parser.add_argument('--z', type=float, default=-1.5, help='Target Z position [m] (NED)')
    parser.add_argument('--rate', type=float, default=10.0, help='Publishing rate [Hz]')
    parser.add_argument('--mode', type=str, default='static', 
                        choices=['static', 'circle', 'line'],
                        help='Target motion mode')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    node = TargetPublisher(
        target_x=parsed_args.x,
        target_y=parsed_args.y,
        target_z=parsed_args.z,
        rate_hz=parsed_args.rate,
        mode=parsed_args.mode
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

