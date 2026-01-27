#include "track_test/vicon_to_target_node.hpp"
#include <cmath>

ViconToTargetNode::ViconToTargetNode()
  : Node("vicon_to_target_node")
  , has_position_data_(false)
{
  // 声明并获取参数
  vicon_topic_name_ = this->declare_parameter("vicon_topic_name", "/vicon/pose");
  vicon_topic_type_ = this->declare_parameter("vicon_topic_type", "PoseStamped");  // "PoseStamped" 或 "TransformStamped"
  velocity_calc_window_ = this->declare_parameter("velocity_calc_window", 0.1);  // 速度计算时间窗口 [s]
  max_history_size_ = this->declare_parameter("max_history_size", 50);  // 位置历史最大长度
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Vicon to Target Converter Node ===");
  RCLCPP_INFO(this->get_logger(), "Vicon topic name: %s", vicon_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Vicon topic type: %s", vicon_topic_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "Velocity calculation window: %.3f s", velocity_calc_window_);
  RCLCPP_INFO(this->get_logger(), "Max position history size: %d", max_history_size_);
  
  // 创建发布者
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/target/position", 
    rclcpp::QoS(10));
    
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/target/velocity", 
    rclcpp::QoS(10));
  
  // 根据话题类型创建订阅者
  if (vicon_topic_type_ == "PoseStamped") {
    vicon_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vicon_topic_name_,
      rclcpp::QoS(10),
      std::bind(&ViconToTargetNode::vicon_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to PoseStamped topic: %s", vicon_topic_name_.c_str());
  } else if (vicon_topic_type_ == "TransformStamped") {
    vicon_transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      vicon_topic_name_,
      rclcpp::QoS(10),
      std::bind(&ViconToTargetNode::vicon_transform_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to TransformStamped topic: %s", vicon_topic_name_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), 
                 "Invalid vicon_topic_type: %s. Must be 'PoseStamped' or 'TransformStamped'",
                 vicon_topic_type_.c_str());
    throw std::runtime_error("Invalid vicon_topic_type parameter");
  }
  
  RCLCPP_INFO(this->get_logger(), "Publishing to /target/position and /target/velocity");
  RCLCPP_INFO(this->get_logger(), "Ready to convert Vicon data to target topics");
}

void ViconToTargetNode::vicon_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 提取位置信息
  geometry_msgs::msg::Point current_pos = msg->pose.position;
  rclcpp::Time current_time = msg->header.stamp;
  
  // 发布位置
  publish_target_position(current_pos.x, current_pos.y, current_pos.z);
  
  // 计算并发布速度
  calculate_velocity_from_position(current_pos, current_time);
}

void ViconToTargetNode::vicon_transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  // 从Transform中提取位置信息
  geometry_msgs::msg::Point current_pos;
  current_pos.x = msg->transform.translation.x;
  current_pos.y = msg->transform.translation.y;
  current_pos.z = msg->transform.translation.z;
  rclcpp::Time current_time = msg->header.stamp;
  
  // 发布位置
  publish_target_position(current_pos.x, current_pos.y, current_pos.z);
  
  // 计算并发布速度
  calculate_velocity_from_position(current_pos, current_time);
}

void ViconToTargetNode::publish_target_position(double x, double y, double z)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;
  
  position_pub_->publish(msg);
}

void ViconToTargetNode::publish_target_velocity(double vx, double vy, double vz)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.twist.linear.x = vx;
  msg.twist.linear.y = vy;
  msg.twist.linear.z = vz;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
  
  velocity_pub_->publish(msg);
}

void ViconToTargetNode::calculate_velocity_from_position(
    const geometry_msgs::msg::Point& current_pos,
    const rclcpp::Time& current_time)
{
  // 添加当前位置到历史记录
  PositionHistory hist;
  hist.position = current_pos;
  hist.timestamp = current_time;
  position_history_.push_back(hist);
  
  // 限制历史记录长度
  while (static_cast<int>(position_history_.size()) > max_history_size_) {
    position_history_.pop_front();
  }
  
  // 如果历史记录不足，无法计算速度
  if (position_history_.size() < 2) {
    publish_target_velocity(0.0, 0.0, 0.0);
    return;
  }
  
  // 找到时间窗口内的最早位置
  rclcpp::Time window_start = current_time - rclcpp::Duration::from_seconds(velocity_calc_window_);
  
  PositionHistory* oldest_in_window = nullptr;
  for (auto& hist : position_history_) {
    if (hist.timestamp >= window_start) {
      oldest_in_window = &hist;
      break;
    }
  }
  
  // 如果没有找到窗口内的数据，使用最早的数据
  if (oldest_in_window == nullptr) {
    oldest_in_window = &position_history_.front();
  }
  
  // 计算时间差
  double dt = (current_time - oldest_in_window->timestamp).seconds();
  
  // 避免除零和过小的时间差
  if (dt < 0.001) {
    publish_target_velocity(0.0, 0.0, 0.0);
    return;
  }
  
  // 计算速度（位置差分）
  double vx = (current_pos.x - oldest_in_window->position.x) / dt;
  double vy = (current_pos.y - oldest_in_window->position.y) / dt;
  double vz = (current_pos.z - oldest_in_window->position.z) / dt;
  
  // 发布速度
  publish_target_velocity(vx, vy, vz);
  
  // 调试日志（限流）
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Vicon->Target: pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f] m/s (dt=%.3f s)",
                       current_pos.x, current_pos.y, current_pos.z,
                       vx, vy, vz, dt);
}

