#include "track_test/target_generator.hpp"
#include <cmath>

TargetGenerator::TargetGenerator(
  rclcpp::Node* node,
  bool use_target_topic,
  const std::string& position_topic,
  const std::string& velocity_topic)
  : node_(node)
  , use_target_topic_(use_target_topic)
  , position_topic_(position_topic)
  , velocity_topic_(velocity_topic)
  , target_x_(0.0)
  , target_y_(0.0)
  , target_z_(0.0)
  , target_vx_(0.0)
  , target_vy_(0.0)
  , target_vz_(0.0)
  , target_ready_(false)
{
  if (use_target_topic_) {
    // 话题模式: 创建订阅者
    RCLCPP_INFO(node_->get_logger(), 
                "TargetGenerator: 话题模式 - 订阅目标位置话题: %s", 
                position_topic_.c_str());
    
    position_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      position_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetGenerator::position_callback, this, std::placeholders::_1));
    
    // 速度话题是可选的
    RCLCPP_INFO(node_->get_logger(), 
                "TargetGenerator: 订阅目标速度话题(可选): %s", 
                velocity_topic_.c_str());
    
    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetGenerator::velocity_callback, this, std::placeholders::_1));
  } else {
    // 静态模式: 不创建订阅者
    RCLCPP_INFO(node_->get_logger(), 
                "TargetGenerator: 静态模式 - 将在初始化时生成固定目标");
  }
}

void TargetGenerator::initialize_static_target(
  double drone_x, 
  double drone_y, 
  double drone_z, 
  double drone_yaw,
  double offset_distance)
{
  if (use_target_topic_) {
    RCLCPP_WARN(node_->get_logger(), 
                "TargetGenerator: 话题模式下调用initialize_static_target将被忽略");
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  
  // 计算无人机正前方的位置
  // 在NED坐标系中，yaw=0指向北(+x方向)
  // 目标位置 = 当前位置 + 偏移量 * [cos(yaw), sin(yaw), 0]
  target_x_ = drone_x + offset_distance * std::cos(drone_yaw);
  target_y_ = drone_y + offset_distance * std::sin(drone_yaw);
  target_z_ = drone_z;  // 保持相同高度
  
  // 静止目标，速度为0
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_vz_ = 0.0;
  
  target_ready_ = true;
  
  RCLCPP_INFO(node_->get_logger(), 
              "TargetGenerator: 静态目标已初始化");
  RCLCPP_INFO(node_->get_logger(), 
              "  无人机位置: [%.2f, %.2f, %.2f], yaw=%.2f rad", 
              drone_x, drone_y, drone_z, drone_yaw);
  RCLCPP_INFO(node_->get_logger(), 
              "  目标位置: [%.2f, %.2f, %.2f] (正前方 %.2f m)", 
              target_x_, target_y_, target_z_, offset_distance);
}

void TargetGenerator::get_target_position(double& target_x, double& target_y, double& target_z) const
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_x = target_x_;
  target_y = target_y_;
  target_z = target_z_;
}

void TargetGenerator::get_target_velocity(double& target_vx, double& target_vy, double& target_vz) const
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_vx = target_vx_;
  target_vy = target_vy_;
  target_vz = target_vz_;
}

bool TargetGenerator::is_target_ready() const
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  return target_ready_;
}

void TargetGenerator::reset()
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_x_ = 0.0;
  target_y_ = 0.0;
  target_z_ = 0.0;
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_vz_ = 0.0;
  target_ready_ = false;
  
  RCLCPP_INFO(node_->get_logger(), "TargetGenerator: 已重置");
}

// ROS2 topic callbacks (话题模式)
void TargetGenerator::position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_x_ = msg->point.x;
  target_y_ = msg->point.y;
  target_z_ = msg->point.z;
  target_ready_ = true;
  
  RCLCPP_DEBUG(node_->get_logger(), 
               "TargetGenerator: 收到目标位置 [%.2f, %.2f, %.2f]",
               target_x_, target_y_, target_z_);
}

void TargetGenerator::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_vx_ = msg->twist.linear.x;
  target_vy_ = msg->twist.linear.y;
  target_vz_ = msg->twist.linear.z;
  
  RCLCPP_DEBUG(node_->get_logger(), 
               "TargetGenerator: 收到目标速度 [%.2f, %.2f, %.2f]",
               target_vx_, target_vy_, target_vz_);
}

