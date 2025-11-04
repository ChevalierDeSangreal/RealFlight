#include "traj_test/traj_test_node.hpp"
#include <cmath>

TrajTestNode::TrajTestNode(int drone_id)
  : Node("traj_test_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , current_state_(FsmState::INIT)
  , waiting_traj_(false)
  , traj_command_sent_(false)
  , traj_started_(false)
  , current_x_(0.0)           
  , current_y_(0.0)           
  , current_z_(0.0)           
  , odom_ready_(false)        
{
  // Declare and get parameters
  circle_radius_    = this->declare_parameter("circle_radius", 2.0);
  circle_duration_  = this->declare_parameter("circle_duration", 20.0);
  ramp_up_time_     = this->declare_parameter("ramp_up_time", 3.0);
  ramp_down_time_   = this->declare_parameter("ramp_down_time", 3.0);
  initial_x_        = this->declare_parameter("initial_x", 0.0);
  initial_y_        = this->declare_parameter("initial_y", 0.0);
  initial_z_        = this->declare_parameter("initial_z", -1.5);  // NED frame
  timer_period_     = this->declare_parameter("timer_period", 0.02);
  
  // NEW: Maximum speed parameter
  max_speed_        = this->declare_parameter("max_speed", -1.0);  // -1 means no limit
  use_max_speed_    = (max_speed_ > 0.0);
  
  // Get PX4 namespace
  px4_namespace_ = get_px4_namespace(drone_id_);
  
  // Calculate effective duration considering max_speed
  effective_duration_ = calculate_effective_duration();
  
  RCLCPP_INFO(this->get_logger(), 
              "Initializing trajectory test node for drone %d", drone_id_);
  RCLCPP_INFO(this->get_logger(), 
              "Circle radius: %.2f m, duration: %.2f s", 
              circle_radius_, circle_duration_);
  
  if (use_max_speed_) {
    double nominal_speed = (2.0 * M_PI * circle_radius_) / circle_duration_;
    RCLCPP_INFO(this->get_logger(), 
                "Max speed limit: %.2f m/s (nominal: %.2f m/s)", 
                max_speed_, nominal_speed);
    if (effective_duration_ != circle_duration_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Duration adjusted to %.2f s to respect max_speed limit", 
                  effective_duration_);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
              "PX4 namespace: %s", px4_namespace_.c_str());
  
  // Publishers
  traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_namespace_ + "in/trajectory_setpoint", 
    rclcpp::QoS(10));
    
  state_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/state/command_drone_" + std::to_string(drone_id_), 
    rclcpp::QoS(10));
  
  // Subscribers
  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/state/state_drone_" + std::to_string(drone_id_),
    rclcpp::QoS(10),
    std::bind(&TrajTestNode::state_callback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_namespace_ + "out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajTestNode::odom_callback, this, std::placeholders::_1));
  
  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_),
    std::bind(&TrajTestNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Trajectory test node initialized");
}

std::string TrajTestNode::get_px4_namespace(int drone_id)
{
  if (drone_id == 0) {
    return "/fmu/";
  } else {
    return "/px4_" + std::to_string(drone_id) + "/fmu/";
  }
}

// NEW: Calculate effective duration based on max_speed constraint
double TrajTestNode::calculate_effective_duration()
{
  if (!use_max_speed_) {
    // No speed limit, use configured duration
    return circle_duration_;
  }
  
  // Calculate the minimum duration required to respect max_speed
  // Tangential velocity: v = ω * r = (2π/T) * r
  // To limit v ≤ v_max: T ≥ (2π * r) / v_max
  double circumference = 2.0 * M_PI * circle_radius_;
  double min_duration = circumference / max_speed_;
  
  // Use the larger of configured duration or minimum required duration
  if (min_duration > circle_duration_) {
    return min_duration;
  }
  
  return circle_duration_;
}

void TrajTestNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto state = static_cast<FsmState>(msg->data);

  // first time HOVER detected
  if (state == FsmState::HOVER &&
      !waiting_traj_ &&
      !traj_command_sent_ &&
      !traj_started_) {
    waiting_traj_ = true;
    hover_detect_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "HOVER detected, will start trajectory in 2.0s");
  }

  if (waiting_traj_ && !traj_started_ && 
      (this->now() - hover_detect_time_).seconds() > 2.0) {
    RCLCPP_INFO(this->get_logger(), "Commanding state machine to TRAJ state");
    send_state_command(static_cast<int>(FsmState::TRAJ));
    traj_command_sent_ = true;
    traj_start_time_ = this->now();
    // hover_detect_time_ = rclcpp::Time(0);  // reset
    // waiting_traj_ = false;
  }

  // TRAJ state detected
  if (state == FsmState::TRAJ && waiting_traj_ && !traj_started_) {
    traj_started_ = true;
    waiting_traj_ = false;
    traj_command_sent_ = false;  // clear
    RCLCPP_INFO(this->get_logger(),
                "FSM entered TRAJ, trajectory start running");
  }

  // started but exited, reset
  if (traj_started_ && state != FsmState::TRAJ) {
    traj_started_ = false;
    waiting_traj_ = false;
    traj_command_sent_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Left TRAJ state, resetting");
  }
}

void TrajTestNode::odom_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  odom_ready_ = true;
}

void TrajTestNode::timer_callback()
{
  if (!odom_ready_) {
    RCLCPP_INFO(this->get_logger(),
                "Failed to get odometry!");
    return;
  }
  
  // Only generate trajectory in TRAJ state
  if (current_state_ == FsmState::TRAJ && traj_started_) {
    double elapsed = (this->now() - traj_start_time_).seconds();
    
    // Check if trajectory is complete (using effective_duration_)
    double total_time = ramp_up_time_ + effective_duration_ + ramp_down_time_;
    if (elapsed >= total_time) {
      RCLCPP_INFO(this->get_logger(), "Circular trajectory complete");
      send_state_command(static_cast<int>(FsmState::END_TRAJ));
      traj_started_ = false;
      return;
    }
    
    // Generate trajectory point
    generate_circular_trajectory(elapsed);
    RCLCPP_INFO(this->get_logger(), "sending trajectory...");
  }
}

void TrajTestNode::generate_circular_trajectory(double t)
{
  double x, y, z, vx, vy, vz, yaw;
  
  // Phase determination
  double phase_time = t;
  double speed_factor = 1.0;
  
  // Ramp up phase
  if (t < ramp_up_time_) {
    speed_factor = t / ramp_up_time_;  // Linear ramp from 0 to 1
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ramp up phase: %.1f%%", speed_factor * 100.0);
  }
  // Constant speed phase (using effective_duration_)
  else if (t < ramp_up_time_ + effective_duration_) {
    speed_factor = 1.0;
    phase_time = t - ramp_up_time_;
  }
  // Ramp down phase
  else {
    double ramp_down_elapsed = t - ramp_up_time_ - effective_duration_;
    speed_factor = 1.0 - (ramp_down_elapsed / ramp_down_time_);
    speed_factor = std::max(0.0, speed_factor);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ramp down phase: %.1f%%", speed_factor * 100.0);
  }
  
  // Angular position (counter-clockwise in NED frame)
  // Use effective_duration_ instead of circle_duration_
  double omega = 2.0 * M_PI / effective_duration_;
  double theta = omega * phase_time * speed_factor;
  
  // Circular trajectory centered at initial position
  // In NED frame: X=North, Y=East, Z=Down
  x = initial_x_ + circle_radius_ * std::cos(theta);
  y = initial_y_ + circle_radius_ * std::sin(theta);
  z = initial_z_;  // Maintain constant altitude
  
  // Velocity (tangent to circle)
  double v_mag = omega * circle_radius_ * speed_factor;
  vx = -v_mag * std::sin(theta);
  vy =  v_mag * std::cos(theta);
  vz = 0.0;
  
  // Yaw (tangent direction)
  yaw = theta + M_PI / 2.0;
  
  // Wrap yaw to [-pi, pi]
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  
  // Publish trajectory setpoint
  publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw);
  
  // Debug output (throttled) - show actual speed
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Traj t=%.1fs: pos=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f], speed=%.2f m/s, yaw=%.2f°",
                       t, x, y, z, vx, vy, v_mag, yaw * 180.0 / M_PI);
}

void TrajTestNode::publish_trajectory_setpoint(
  double x, double y, double z,
  double vx, double vy, double vz,
  double yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  
  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;
  
  msg.velocity[0] = vx;
  msg.velocity[1] = vy;
  msg.velocity[2] = vz;
  
  msg.yaw = yaw;
  
  msg.timestamp = this->now().nanoseconds() / 1000;
  
  traj_pub_->publish(msg);
}

void TrajTestNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}