#include "track_test/target_publisher_node.hpp"
#include <cmath>
#include <chrono>

TargetPublisherNode::TargetPublisherNode()
  : Node("target_publisher_node")
  , trajectory_started_(false)
  , drone_id_(0)
  , current_state_(FsmState::INIT)
  , in_traj_state_(false)
{
  // 声明并获取参数
  drone_id_        = this->declare_parameter("drone_id", 0);
  circle_radius_    = this->declare_parameter("circle_radius", 2.0);
  circle_duration_  = this->declare_parameter("circle_duration", 20.0);
  circle_init_phase_= this->declare_parameter("circle_init_phase", 0.0);
  circle_times_     = this->declare_parameter("circle_times", 1);
  ramp_up_time_     = this->declare_parameter("ramp_up_time", 3.0);
  ramp_down_time_   = this->declare_parameter("ramp_down_time", 3.0);
  stationary_time_  = this->declare_parameter("stationary_time", 3.0);
  
  circle_center_x_  = this->declare_parameter("circle_center_x", 0.0);
  circle_center_y_  = this->declare_parameter("circle_center_y", 0.0);
  circle_center_z_  = this->declare_parameter("circle_center_z", -1.2);
  
  timer_period_     = this->declare_parameter("timer_period", 0.02);
  max_speed_        = this->declare_parameter("max_speed", -1.0);
  use_max_speed_    = (max_speed_ > 0.0);
  
  // 获取use_sim_time参数，决定使用ROS时钟还是系统时钟
  // use_sim_time是ROS2标准参数，通常由launch文件自动声明，使用get_parameter获取
  if (this->has_parameter("use_sim_time")) {
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
  } else {
    // 如果参数不存在，使用默认值false（实机模式）
    use_sim_time_ = false;
    RCLCPP_WARN(this->get_logger(), 
                "use_sim_time parameter not found, defaulting to false (system clock)");
  }
  
  // Parameter validation
  if (circle_times_ < 1) {
    RCLCPP_WARN(this->get_logger(), 
                "circle_times must be >= 1, setting to 1");
    circle_times_ = 1;
  }
  
  // 计算轨迹参数
  effective_duration_ = calculate_effective_duration();
  max_angular_vel_ = 2.0 * M_PI / effective_duration_;
  angular_acceleration_ = max_angular_vel_ / ramp_up_time_;
  
  double theta_ramp_up = 0.5 * max_angular_vel_ * ramp_up_time_;
  double theta_ramp_down = 0.5 * max_angular_vel_ * ramp_down_time_;
  double theta_ramps_total = theta_ramp_up + theta_ramp_down;
  
  // N圈所需的总角位移
  double theta_required = circle_times_ * 2.0 * M_PI;
  // 匀速阶段所需的角位移
  double theta_constant = theta_required - theta_ramps_total;
  // 匀速阶段所需的时间
  total_constant_duration_ = theta_constant / max_angular_vel_;
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Target Publisher Node ===");
  RCLCPP_INFO(this->get_logger(), "Circle radius: %.2f m", circle_radius_);
  RCLCPP_INFO(this->get_logger(), "Circle center: [%.2f, %.2f, %.2f] m (NED)", 
              circle_center_x_, circle_center_y_, circle_center_z_);
  RCLCPP_INFO(this->get_logger(), "Initial stationary time: %.2f s", stationary_time_);
  RCLCPP_INFO(this->get_logger(), "Number of circles: %d", circle_times_);
  RCLCPP_INFO(this->get_logger(), "Motion duration: %.2f s", 
              ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
  RCLCPP_INFO(this->get_logger(), "Total duration: %.2f s", 
              stationary_time_ + ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
  RCLCPP_INFO(this->get_logger(), "Publish frequency: %.0f Hz", 1.0/timer_period_);
  RCLCPP_INFO(this->get_logger(), "Clock mode: %s", 
              use_sim_time_ ? "SIM_TIME (ROS clock)" : "SYSTEM_TIME (steady_clock)");
  RCLCPP_INFO(this->get_logger(), "Drone ID: %d", drone_id_);
  RCLCPP_INFO(this->get_logger(), "State topic: /state/state_drone_%d", drone_id_);
  RCLCPP_INFO(this->get_logger(), "Target publishing will start when entering TRAJ state");
  
  // 创建发布者
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/target/position", 
    rclcpp::QoS(10));
    
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/target/velocity", 
    rclcpp::QoS(10));
  
  // 创建状态订阅者（监听状态机状态）
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    state_topic,
    rclcpp::QoS(10),
    std::bind(&TargetPublisherNode::state_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribed to state topic: %s", state_topic.c_str());
  
  // 创建定时器
  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timer_period_)
      )),
      std::bind(&TargetPublisherNode::timer_callback, this));
  
  // 注意：不在构造函数中初始化 start_time_
  // 而是在进入 TRAJ 状态后，在 timer_callback 中初始化，确保时钟已经同步
  // 这样可以避免在 use_sim_time=true 时，时钟话题还未订阅到导致的时间错误
  trajectory_started_ = false;  // 初始化为 false，等待进入 TRAJ 状态后初始化
  
  RCLCPP_INFO(this->get_logger(), "Target trajectory generation will start when entering TRAJ state");
}

double TargetPublisherNode::calculate_effective_duration()
{
  if (!use_max_speed_) {
    return circle_duration_;
  }
  
  double circumference = 2.0 * M_PI * circle_radius_;
  double min_duration = circumference / max_speed_;
  
  if (min_duration > circle_duration_) {
    return min_duration;
  }
  
  return circle_duration_;
}

void TargetPublisherNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto state = static_cast<FsmState>(msg->data);
  current_state_ = state;
  
  // 检测进入 TRAJ 状态
  if (state == FsmState::TRAJ && !in_traj_state_) {
    in_traj_state_ = true;
    RCLCPP_INFO(this->get_logger(), 
                "✅ Entered TRAJ state - Target trajectory publishing will start");
    // 不在这里初始化时间，而是在 timer_callback 中初始化以确保时钟同步
  }
  
  // 检测退出 TRAJ 状态
  if (in_traj_state_ && state != FsmState::TRAJ) {
    in_traj_state_ = false;
    trajectory_started_ = false;  // 重置，以便下次进入 TRAJ 状态时重新初始化
    RCLCPP_INFO(this->get_logger(), 
                "Left TRAJ state - Target trajectory publishing stopped");
  }
}

double TargetPublisherNode::calculate_theta_at_time(double t)
{
  double theta = 0.0;
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_; 
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;  
  double t_down = ramp_down_time_;
  
  // 阶段 1: 加速
  if (t <= t_up) {
    theta = 0.5 * alpha * t * t;
  }
  // 阶段 2: 匀速
  else if (t <= t_up + t_const) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double dt = t - t_up;
    theta = theta_at_t_up + omega_max * dt;
  }
  // 阶段 3: 减速
  else if (t <= t_up + t_const + t_down) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    
    double t_start_down = t_up + t_const;
    double dt = t - t_start_down;
    theta = theta_at_start_down + omega_max * dt - 0.5 * alpha_down * dt * dt;
  }
  // 阶段 4: 保持最终位置
  else {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    theta = theta_at_start_down + omega_max * t_down - 0.5 * alpha_down * t_down * t_down;
  }
  
  return theta;
}

double TargetPublisherNode::calculate_angular_velocity_at_time(double t)
{
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  double t_start_down = t_up + t_const;
  
  double current_omega = 0.0;

  if (t <= t_up) {
    // 加速阶段
    current_omega = alpha * t;
  }
  else if (t <= t_start_down) {
    // 匀速阶段
    current_omega = omega_max;
  }
  else if (t <= t_start_down + t_down) {
    // 减速阶段
    double dt_down = t - t_start_down;
    current_omega = omega_max - alpha_down * dt_down;
    current_omega = std::max(0.0, current_omega);
  }
  else {
    // 运动完成
    current_omega = 0.0;
  }
  
  return current_omega;
}

void TargetPublisherNode::timer_callback()
{
  // 未进入 TRAJ 状态时，发布初始静止位置（避免死锁）
  if (!in_traj_state_) {
    // 发布初始静止位置，让 track_test_node 能够检测到目标并进入 TRAJ 状态
    double theta_initial = circle_init_phase_;
    double x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
    double y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
    double z = circle_center_z_;
    
    // 零速度
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    
    // 发布位置和速度
    publish_target_position(x, y, z);
    publish_target_velocity(vx, vy, vz);
    
    // 定期打印日志
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[WAITING] Publishing initial stationary target at [%.2f, %.2f, %.2f] (waiting for TRAJ state)",
                         x, y, z);
    return;
  }
  
  // 在进入 TRAJ 状态后的第一次回调时初始化开始时间（确保时钟已同步）
  if (!trajectory_started_) {
    if (use_sim_time_) {
      // SITL模式：使用ROS时钟（模拟时间）
      // 等待时钟话题可用后再获取时间（参考offboard_fsm_node.cpp的实现方式）
      start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Using ROS clock (sim_time) for trajectory timing");
      RCLCPP_INFO(this->get_logger(), "start_time_ = %.9f seconds", start_time_.seconds());
    } else {
      // Onboard模式：使用系统时钟（steady_clock）
      start_time_system_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(this->get_logger(), "Using system clock (steady_clock) for trajectory timing");
    }
    trajectory_started_ = true;
    RCLCPP_INFO(this->get_logger(), "✅ Target trajectory generation started - starting timer (stationary for %.1fs, then circular motion)", stationary_time_);
    return;  // 第一次回调只初始化，不生成轨迹
  }
  
  double elapsed = 0.0;
  
  if (use_sim_time_) {
    // SITL模式：使用ROS时钟（模拟时间）
    // 参考offboard_fsm_node.cpp的实现方式
    elapsed = (this->now() - start_time_).seconds();
    
    // 处理时钟抖动导致的负时间（参考MJerkSegment::sample的实现）
    if (elapsed < -0.01) {
      elapsed = 0.0;
    }
    elapsed = std::max(0.0, elapsed);
  } else {
    // Onboard模式：使用系统时钟（steady_clock）
    auto current_time_system = std::chrono::steady_clock::now();
    auto elapsed_duration = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time_system - start_time_system_);
    elapsed = elapsed_duration.count();
  }
  
  generate_circular_trajectory(elapsed);
}

void TargetPublisherNode::generate_circular_trajectory(double t)
{
  double x, y, z, vx, vy, vz;
  
  // Phase 1: Stationary period - stay at initial position
  if (t < stationary_time_) {
    // Calculate initial position (starting point on circle)
    double theta_initial = circle_init_phase_;
    x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
    y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
    z = circle_center_z_;
    
    // Zero velocity during stationary period
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    
    // Publish position and velocity
    publish_target_position(x, y, z);
    publish_target_velocity(vx, vy, vz);
    
    // Debug log
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[STATIONARY] t=%.1f/%.1fs | position=[%.2f, %.2f, %.2f] | velocity=0.00 m/s",
                         t, stationary_time_, x, y, z);
    return;
  }
  
  // Phase 2: Circular motion - calculate motion time offset by stationary period
  double motion_time = t - stationary_time_;
  
  // Calculate angular velocity at motion time
  double current_omega = calculate_angular_velocity_at_time(motion_time);
  
  // Calculate angular position at motion time
  double theta = calculate_theta_at_time(motion_time);
  double theta_with_phase = theta + circle_init_phase_;
  
  // Position on circle
  x = circle_center_x_ + circle_radius_ * std::cos(theta_with_phase);
  y = circle_center_y_ + circle_radius_ * std::sin(theta_with_phase);
  z = circle_center_z_;
  
  // Linear velocity in tangential direction
  double v_linear = current_omega * circle_radius_;
  vx = -v_linear * std::sin(theta_with_phase);
  vy =  v_linear * std::cos(theta_with_phase);
  vz = 0.0;
  
  // Publish position and velocity
  publish_target_position(x, y, z);
  publish_target_velocity(vx, vy, vz);
  
  // Debug log
  double total_motion_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  std::string phase;
  double current_circle = 0.0;
  
  if (motion_time <= t_up) {
    phase = "ACCELERATING";
    current_circle = 0.0;
  } else if (motion_time <= t_up + t_const) {
    phase = "CONSTANT";
    double elapsed_const = motion_time - t_up;
    current_circle = elapsed_const / effective_duration_;
  } else if (motion_time <= total_motion_time) {
    phase = "DECELERATING";
    current_circle = circle_times_;
  } else {
    phase = "COMPLETE";
    current_circle = circle_times_;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "[%s] t=%.1fs (motion: %.1fs) | circle=%.2f/%d | position=[%.2f, %.2f, %.2f] | velocity=%.2f m/s",
                       phase.c_str(), t, motion_time, current_circle, circle_times_, 
                       x, y, z, v_linear);
}

void TargetPublisherNode::publish_target_position(double x, double y, double z)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";  // 或者 "world", "odom" 等，根据实际坐标系
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;
  
  position_pub_->publish(msg);
}

void TargetPublisherNode::publish_target_velocity(double vx, double vy, double vz)
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

