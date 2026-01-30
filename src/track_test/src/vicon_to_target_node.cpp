#include "track_test/vicon_to_target_node.hpp"
#include <cmath>

ViconToTargetNode::ViconToTargetNode()
  : Node("vicon_to_target_node")
  , has_position_data_(false)
{
  // 声明并获取参数
  px4_topic_name_ = this->declare_parameter("px4_topic_name", "/px4_3/fmu/out/vehicle_odometry");
  velocity_calc_window_ = this->declare_parameter("velocity_calc_window", 0.1);  // 速度计算时间窗口 [s]
  max_history_size_ = this->declare_parameter("max_history_size", 50);  // 位置历史最大长度
  
  RCLCPP_INFO(this->get_logger(), 
              "=== PX4 to Target Converter Node ===");
  RCLCPP_INFO(this->get_logger(), "PX4 topic name: %s", px4_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Velocity calculation window: %.3f s", velocity_calc_window_);
  RCLCPP_INFO(this->get_logger(), "Max position history size: %d", max_history_size_);
  
  // 创建发布者
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/target/position", 
    rclcpp::QoS(10));
    
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/target/velocity", 
    rclcpp::QoS(10));
  
  // 创建订阅者（只支持VehicleOdometry）
  px4_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_topic_name_,
    rclcpp::SensorDataQoS(),
    std::bind(&ViconToTargetNode::px4_odometry_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to VehicleOdometry topic: %s", px4_topic_name_.c_str());
  
  RCLCPP_INFO(this->get_logger(), "Publishing to /target/position and /target/velocity");
  RCLCPP_INFO(this->get_logger(), "Ready to convert PX4 odometry data to target topics");
}

void ViconToTargetNode::px4_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // 从VehicleOdometry中提取位置信息（PX4使用NED坐标系）
  geometry_msgs::msg::Point current_pos;
  current_pos.x = msg->position[0];  // North (NED)
  current_pos.y = msg->position[1];  // East (NED)
  current_pos.z = msg->position[2];  // Down (NED)
  
  // 使用节点时钟获取当前时间（更可靠）
  rclcpp::Time current_time = this->now();
  
  // 发布位置
  publish_target_position(current_pos.x, current_pos.y, current_pos.z);
  
  // 如果有速度信息且有效，直接使用；否则通过位置差分计算
  if (!std::isnan(msg->velocity[0]) && !std::isnan(msg->velocity[1]) && !std::isnan(msg->velocity[2])) {
    // PX4速度也是NED坐标系
    publish_target_velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  } else {
    // 速度无效，通过位置差分计算
    calculate_velocity_from_position(current_pos, current_time);
  }
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

