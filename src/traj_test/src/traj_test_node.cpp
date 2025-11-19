#include "traj_test/traj_test_node.hpp"
#include <cmath>
#include <algorithm>

// Observation space bounds (matching TrackEnvVer5/Ver6 training config)
// obs: [v_body(3), g_body(3), target_pos_body(3)]
namespace {
  constexpr float OBS_MIN[9] = {
    -20.0f, -20.0f, -20.0f,    // v_body (body-frame velocity)
    -1.0f, -1.0f, -1.0f,       // g_body (body-frame gravity direction)
    -100.0f, -100.0f, -100.0f  // target_pos_body (body-frame target position, relative)
  };
  
  constexpr float OBS_MAX[9] = {
    20.0f, 20.0f, 20.0f,       // v_body
    1.0f, 1.0f, 1.0f,          // g_body
    100.0f, 100.0f, 100.0f     // target_pos_body
  };
  
  // Normalize observation to [-1, 1] range (matching Python training code)
  void normalize_observation(std::vector<float>& obs) {
    for (size_t i = 0; i < obs.size() && i < 9; ++i) {
      obs[i] = 2.0f * (obs[i] - OBS_MIN[i]) / (OBS_MAX[i] - OBS_MIN[i]) - 1.0f;
      // Clamp to [-1, 1] to handle edge cases
      obs[i] = std::clamp(obs[i], -1.0f, 1.0f);
    }
  }
}

TrajTestNode::TrajTestNode(int drone_id)
  : Node("traj_test_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , current_state_(FsmState::INIT)
  , waiting_hover_(false)
  , hover_command_sent_(false)
  , hover_started_(false)
  , hover_completed_(false)
  , accumulated_elapsed_(0.0)
  , hover_x_(0.0)
  , hover_y_(0.0)
  , hover_z_(0.0)
  , hover_yaw_(0.0)
  , current_x_(0.0)           
  , current_y_(0.0)           
  , current_z_(0.0)           
  , odom_ready_(false)
  , current_vx_(0.0)
  , current_vy_(0.0)
  , current_vz_(0.0)
  , current_roll_(0.0)
  , current_pitch_(0.0)
  , current_yaw_(0.0)
  , current_roll_rate_(0.0)
  , current_pitch_rate_(0.0)
  , current_yaw_rate_(0.0)
  , local_position_ready_(false)
  , attitude_ready_(false)
  , target_x_(0.0)
  , target_y_(0.0)
  , target_z_(2.0)
  , target_vx_(0.0)
  , target_vy_(0.0)
  , target_vz_(0.0)
  , use_neural_control_(false)
{
  // Parameters
  hover_duration_   = this->declare_parameter("hover_duration", 3.0);  // TRAJ control duration: 3.0s
  hover_thrust_     = this->declare_parameter("hover_thrust", 0.71);
  timer_period_     = this->declare_parameter("timer_period", 0.01);  // 100 Hz to match training
  use_neural_control_ = this->declare_parameter("use_neural_control", true);
  model_path_       = this->declare_parameter("model_path", "");
  target_x_         = this->declare_parameter("target_x", 0.0);
  target_y_         = this->declare_parameter("target_y", 0.0);
  target_z_         = this->declare_parameter("target_z", 2.0);
  
  // PX4 namespace
  px4_namespace_ = get_px4_namespace(drone_id_);
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Trajectory Test Node for Drone %d (Neural Network Control) ===", drone_id_);
  RCLCPP_INFO(this->get_logger(),
              "Parameters:");
  RCLCPP_INFO(this->get_logger(),
              "  - Hover duration: %.2f s", hover_duration_);
  RCLCPP_INFO(this->get_logger(),
              "  - Use neural control: %s", use_neural_control_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(),
              "  - Control period: %.3f s (%.1f Hz)", timer_period_, 1.0/timer_period_);
  RCLCPP_INFO(this->get_logger(),
              "  - Target: [%.2f, %.2f, %.2f]", target_x_, target_y_, target_z_);
  RCLCPP_INFO(this->get_logger(), 
              "  - PX4 namespace: %s", px4_namespace_.c_str());
  
  // Initialize neural network policy
  if (use_neural_control_) {
    if (model_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Neural control enabled but model_path is empty!");
      use_neural_control_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "  - Model path: %s", model_path_.c_str());
      policy_ = std::make_unique<TFLitePolicyInference>(model_path_);
      if (!policy_->is_initialized()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize neural network policy!");
        use_neural_control_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "  ✅ Neural network policy initialized");
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "  - Fallback mode: hover thrust control");
  }
  RCLCPP_INFO(this->get_logger(), "===================================================");
  
  // Publishers
  rates_pub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
    px4_namespace_ + "in/vehicle_rates_setpoint", 
    rclcpp::QoS(1));
    
  state_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/state/command_drone_" + std::to_string(drone_id_), 
    rclcpp::QoS(10));
  
  // Subscribers
  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/state/state_drone_" + std::to_string(drone_id_),
    rclcpp::QoS(10),
    std::bind(&TrajTestNode::state_callback, this, std::placeholders::_1));
    
  // VehicleOdometry contains position, velocity, and attitude - no need for separate subscriptions!
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_namespace_ + "out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajTestNode::odom_callback, this, std::placeholders::_1));
  
  // Debug: Subscribe to PX4's actual vehicle rates setpoint output
  // This shows what PX4 is actually executing (after internal processing/limiting)
  rates_setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
    px4_namespace_ + "out/vehicle_rates_setpoint",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajTestNode::vehicle_rates_setpoint_callback, this, std::placeholders::_1));
  
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

void TrajTestNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto state = static_cast<FsmState>(msg->data);
  
  // Update current state
  current_state_ = state;

  // first HOVER detection
  if (state == FsmState::HOVER &&
      !waiting_hover_ &&
      !hover_command_sent_ &&
      !hover_started_) {
    waiting_hover_ = true;
    hover_detect_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "HOVER detected, will start hover control in 2.0s");
  }

  if (waiting_hover_ && !hover_started_ && !hover_completed_ &&
      (this->now() - hover_detect_time_).seconds() > 2.0) {
    RCLCPP_INFO(this->get_logger(), "Commanding state machine to TRAJ state for hover control");
    send_state_command(static_cast<int>(FsmState::TRAJ));
    hover_command_sent_ = true;
    hover_start_time_ = this->now();
  }

  // TRAJ detected - capture current position and start trajectory tracking
  if (state == FsmState::TRAJ && waiting_hover_ && !hover_started_) {
    hover_started_ = true;
    waiting_hover_ = false;
    hover_command_sent_ = false;
    hover_start_time_ = this->now();
    accumulated_elapsed_ = 0.0;
    
    // Capture current position for hover
    hover_x_ = current_x_;
    hover_y_ = current_y_;
    hover_z_ = current_z_;
    hover_yaw_ = 0.0;  // Face north
    
    // Reset neural network policy (policy manages its own action-obs buffer)
    if (use_neural_control_ && policy_) {
      // Get initial observation to fill the buffer (must match training!)
      std::vector<float> initial_obs = get_observation();
      
      // Normalize observation to [-1, 1] range (CRITICAL: must match training!)
      normalize_observation(initial_obs);
      
      // Hovering action (normalized): [thrust≈0.42, omega_x=0, omega_y=0, omega_z=0]
      // In training, hovering thrust is normalized to ~0.42 in [-1,1] range
      // (which maps to 0.71 in [0,1] Gazebo range)
      std::vector<float> hovering_action = {0.42f, 0.0f, 0.0f, 0.0f};
      
      policy_->reset(initial_obs, hovering_action);
      RCLCPP_INFO(this->get_logger(), 
                  "Neural network policy reset with normalized initial obs and hovering action");
    }
    
    RCLCPP_INFO(this->get_logger(),
                "🚁 FSM entered TRAJ - Neural control active for %.1f s", hover_duration_);
    RCLCPP_INFO(this->get_logger(),
                "   Start pos: [%.2f, %.2f, %.2f] | Target: [%.2f, %.2f, %.2f]",
                hover_x_, hover_y_, hover_z_, target_x_, target_y_, target_z_);
  }

  // Exited TRAJ, reset
  if (hover_started_ && state != FsmState::TRAJ) {
    hover_started_ = false;
    waiting_hover_ = false;
    hover_command_sent_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Left TRAJ state, resetting");
  }
}

void TrajTestNode::odom_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // Position
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  
  // Velocity
  current_vx_ = msg->velocity[0];
  current_vy_ = msg->velocity[1];
  current_vz_ = msg->velocity[2];
  
  // Attitude (quaternion to Euler angles)
  // q = [w, x, y, z]
  double w = msg->q[0];
  double x = msg->q[1];
  double y = msg->q[2];
  double z = msg->q[3];
  
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  current_roll_ = std::atan2(sinr_cosp, cosr_cosp);
  
  // Pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0)
    current_pitch_ = std::copysign(M_PI / 2.0, sinp);
  else
    current_pitch_ = std::asin(sinp);
  
  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  
  // Mark all data as ready
  odom_ready_ = true;
  local_position_ready_ = true;
  attitude_ready_ = true;
}


// Debug callback: Monitor what PX4 is actually executing
void TrajTestNode::vehicle_rates_setpoint_callback(
  const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
{
  // Only log when in TRAJ state to avoid spam
  if (current_state_ == FsmState::TRAJ && hover_started_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[PX4 EXECUTING] thrust_body=[%.4f, %.4f, %.4f], "
                         "rates=[%.4f, %.4f, %.4f] rad/s",
                         msg->thrust_body[0], msg->thrust_body[1], msg->thrust_body[2],
                         msg->roll, msg->pitch, msg->yaw);
  }
}

void TrajTestNode::timer_callback()
{
  if (!odom_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for odometry...");
    return;
  }
  
  // Only run in TRAJ state
  if (current_state_ == FsmState::TRAJ && hover_started_ && !hover_completed_) {
    accumulated_elapsed_ += timer_period_;
    
    // Check if TRAJ control duration is complete
    if (accumulated_elapsed_ >= hover_duration_) {
      RCLCPP_INFO(this->get_logger(), 
                  "✅ TRAJ control complete (%.1f s) - sending END_TRAJ command", hover_duration_);
      send_state_command(static_cast<int>(FsmState::END_TRAJ));
      hover_completed_ = true;
      return;
    }
    
    // Publish control command
    if (use_neural_control_ && local_position_ready_ && attitude_ready_) {
      // Get action from neural network (policy handles action repeat internally)
      publish_neural_control();
      
      // Log current status (throttled)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Neural control | pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] | elapsed: %.1f/%.1f s",
                           current_x_, current_y_, current_z_,
                           current_vx_, current_vy_, current_vz_,
                           accumulated_elapsed_, hover_duration_);
    } else {
      // Fallback to simple hover
      publish_hover_setpoint();
      
      // Log current status (throttled)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Fallback hover control: thrust=%.3f | elapsed: %.1f/%.1f s",
                           hover_thrust_, accumulated_elapsed_, hover_duration_);
    }
  }
}

// Get observation vector for neural network (9D)
// Based on TrackEnvVer6 observation space - all in body frame
// Observation composition:
// 1. 机体系速度 (3) - quad velocity in body frame
// 2. 机体系重力方向 (3) - gravity direction in body frame
// 3. 机体系目标位置 (3) - target relative position in body frame
std::vector<float> TrajTestNode::get_observation()
{
  std::vector<float> obs(OBS_DIM);
  
  // Compute rotation matrix from Euler angles (ZYX convention)
  // This transforms from body frame to world frame (NED)
  float cr = std::cos(current_roll_);
  float sr = std::sin(current_roll_);
  float cp = std::cos(current_pitch_);
  float sp = std::sin(current_pitch_);
  float cy = std::cos(current_yaw_);
  float sy = std::sin(current_yaw_);
  
  // Rotation matrix R (body to world)
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  float R[3][3];
  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;
  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;
  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
  
  // R^T transforms from world frame to body frame
  // 1. 机体系速度 (v_body = R^T * v_world)
  float v_world[3] = {
    static_cast<float>(current_vx_),
    static_cast<float>(current_vy_),
    static_cast<float>(current_vz_)
  };
  obs[0] = R[0][0] * v_world[0] + R[1][0] * v_world[1] + R[2][0] * v_world[2];
  obs[1] = R[0][1] * v_world[0] + R[1][1] * v_world[1] + R[2][1] * v_world[2];
  obs[2] = R[0][2] * v_world[0] + R[1][2] * v_world[1] + R[2][2] * v_world[2];
  
  // 2. 机体系重力方向 (g_body = R^T * g_world)
  // In NED coordinate system, gravity points down: [0, 0, 1]
  float g_world[3] = {0.0f, 0.0f, 1.0f};
  obs[3] = R[0][0] * g_world[0] + R[1][0] * g_world[1] + R[2][0] * g_world[2];
  obs[4] = R[0][1] * g_world[0] + R[1][1] * g_world[1] + R[2][1] * g_world[2];
  obs[5] = R[0][2] * g_world[0] + R[1][2] * g_world[1] + R[2][2] * g_world[2];
  
  // 3. 机体系目标相对位置 (target_pos_body = R^T * target_pos_world_relative)
  float target_rel_world[3] = {
    static_cast<float>(target_x_ - current_x_),
    static_cast<float>(target_y_ - current_y_),
    static_cast<float>(target_z_ - current_z_)
  };
  obs[6] = R[0][0] * target_rel_world[0] + R[1][0] * target_rel_world[1] + R[2][0] * target_rel_world[2];
  obs[7] = R[0][1] * target_rel_world[0] + R[1][1] * target_rel_world[1] + R[2][1] * target_rel_world[2];
  obs[8] = R[0][2] * target_rel_world[0] + R[1][2] * target_rel_world[1] + R[2][2] * target_rel_world[2];
  
  return obs;
}

// Publish neural network control command
void TrajTestNode::publish_neural_control()
{
  // Get current observation (9D)
  std::vector<float> obs = get_observation();
  
  // Normalize observation to [-1, 1] range (CRITICAL: must match training!)
  normalize_observation(obs);
  
  // Get action from neural network (policy manages buffer and action repeat internally)
  // Note: policy internally repeats action every 10 steps (0.1s at 100Hz)
  std::vector<float> action = policy_->get_action(obs);
  bool is_new_inference = policy_->is_new_inference();
  
  // Create body rate setpoint message
  // Note: OffboardControlMode is managed by offboard_state_machine
  px4_msgs::msg::VehicleRatesSetpoint msg;
  
  // Action output from NN: [thrust, omega_x, omega_y, omega_z]
  // All in range [-1, 1] from tanh activation
  // Note: omega_x=roll_rate, omega_y=pitch_rate, omega_z=yaw_rate
  
  float thrust_raw = action[0];      // Thrust (tanh output)
  float omega_x = action[1];         // Roll rate (body x-axis)
  float omega_y = action[2];         // Pitch rate (body y-axis)
  float omega_z = action[3];         // Yaw rate (body z-axis)
  
  // Set body rates (these are already in rad/s from the network)
  msg.roll = omega_x;    // Roll rate [rad/s]
  msg.pitch = omega_y;   // Pitch rate [rad/s]
  msg.yaw = omega_z;     // Yaw rate [rad/s]
  
  // Thrust mapping: [-1, 1] -> [0, 1]
  float thrust_normalized = (thrust_raw + 1.0f) * 0.5f;
  msg.thrust_body[0] = 0.0f;
  msg.thrust_body[1] = 0.0f;
  msg.thrust_body[2] = -thrust_normalized;  // Negative for upward thrust in NED
  
  // Timestamp
  msg.timestamp = offboard_utils::get_timestamp_us();
  
  // Publish
  rates_pub_->publish(msg);
  
  // Debug: Log neural network output
  // Show whether this is a new inference or repeated action
  if (is_new_inference) {
    // Print every new inference (no throttle)
    RCLCPP_INFO(this->get_logger(),
                "[NEW NN OUTPUT] thrust=%.3f(raw=%.3f), rates=[%.3f, %.3f, %.3f] rad/s",
                thrust_normalized, thrust_raw, omega_x, omega_y, omega_z);
  } else {
    // Log repeated actions at DEBUG level to reduce clutter
    RCLCPP_DEBUG(this->get_logger(),
                 "[REPEAT] thrust=%.3f, rates=[%.3f, %.3f, %.3f] rad/s",
                 thrust_normalized, omega_x, omega_y, omega_z);
  }
}

// Publish hover setpoint using body rate control (zero angular velocity + hover thrust)
void TrajTestNode::publish_hover_setpoint()
{
  // Note: OffboardControlMode is managed by offboard_state_machine
  px4_msgs::msg::VehicleRatesSetpoint msg;
  
  // Body rates: zero for hovering (no rotation)
  msg.roll = 0.0f;   // Roll rate [rad/s]
  msg.pitch = 0.0f;  // Pitch rate [rad/s]
  msg.yaw = 0.0f;    // Yaw rate [rad/s]
  
  // Thrust: normalized thrust to maintain altitude
  // Range: [0.0, 1.0], typically ~0.5 for hover
  msg.thrust_body[0] = 0.0f;  // Forward thrust (not used)
  msg.thrust_body[1] = 0.0f;  // Right thrust (not used)
  msg.thrust_body[2] = static_cast<float>(-hover_thrust_);  // Down thrust (negative = upward)
  
  // Timestamp
  msg.timestamp = offboard_utils::get_timestamp_us();
  
  // Publish
  rates_pub_->publish(msg);
}

void TrajTestNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}