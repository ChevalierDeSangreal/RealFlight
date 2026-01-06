#include "track_test/track_test_node.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <string>

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

TrackTestNode::TrackTestNode(int drone_id)
  : Node("track_test_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , current_state_(FsmState::INIT)
  , waiting_hover_(false)
  , hover_command_sent_(false)
  , hover_started_(false)
  , hover_completed_(false)
  , neural_control_ready_(false)
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
  , target_z_(0.0)
  , target_vx_(0.0)
  , target_vy_(0.0)
  , target_vz_(0.0)
  , use_neural_control_(false)
  , current_action_(4, 0.0f)  // Initialize with zero action [thrust, omega_x, omega_y, omega_z]
  , step_counter_(0)           // Initialize step counter
{
  // Parameters
  hover_duration_   = this->declare_parameter("hover_duration", 3.0);  // TRAJ control duration: 3.0s
  hover_thrust_     = this->declare_parameter("hover_thrust", 0.254);  // Match hover_test (lower thrust for neural control)
  mode_stabilization_delay_ = this->declare_parameter("mode_stabilization_delay", 0.6);  // Wait 0.6s for mode switch
  action_update_period_ = this->declare_parameter("action_update_period", 0.02);  // 50 Hz for NN inference
  control_send_period_  = this->declare_parameter("control_send_period", 0.01);  // 100 Hz for control commands
  use_neural_control_ = this->declare_parameter("use_neural_control", true);
  model_path_       = this->declare_parameter("model_path", "");
  
  // Target generation parameters
  use_target_topic_ = this->declare_parameter("use_target_topic", false);
  target_position_topic_ = this->declare_parameter("target_position_topic", "/target/position");
  target_velocity_topic_ = this->declare_parameter("target_velocity_topic", "/target/velocity");
  target_offset_distance_ = this->declare_parameter("target_offset_distance", 1.0);  // 默认正前方1米
  
  // Initialize target generator
  target_generator_ = std::make_unique<TargetGenerator>(
    this, 
    use_target_topic_,
    target_position_topic_,
    target_velocity_topic_);
  
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
              "  - Mode stabilization delay: %.2f s (wait for body_rate mode)", mode_stabilization_delay_);
  RCLCPP_INFO(this->get_logger(),
              "  - Action update period: %.3f s (%.1f Hz)", action_update_period_, 1.0/action_update_period_);
  RCLCPP_INFO(this->get_logger(),
              "  - Control send period: %.3f s (%.1f Hz)", control_send_period_, 1.0/control_send_period_);
  RCLCPP_INFO(this->get_logger(),
              "  - Target mode: %s", use_target_topic_ ? "ROS2 Topic" : "Static (front of drone)");
  if (use_target_topic_) {
    RCLCPP_INFO(this->get_logger(),
                "    - Position topic: %s", target_position_topic_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "    - Velocity topic: %s", target_velocity_topic_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
                "    - Offset distance: %.2f m (in front of drone)", target_offset_distance_);
  }
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
    std::bind(&TrackTestNode::state_callback, this, std::placeholders::_1));
    
  // VehicleOdometry contains position, velocity, and attitude - no need for separate subscriptions!
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_namespace_ + "out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&TrackTestNode::odom_callback, this, std::placeholders::_1));
  
  // Debug: Subscribe to PX4's actual vehicle rates setpoint output
  // This shows what PX4 is actually executing (after internal processing/limiting)
  rates_setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
    px4_namespace_ + "out/vehicle_rates_setpoint",
    rclcpp::SensorDataQoS(),
    std::bind(&TrackTestNode::vehicle_rates_setpoint_callback, this, std::placeholders::_1));
  
  // Two-timer architecture to avoid accumulated timing errors:
  // 1. Action update timer (50Hz): Neural network inference
  // 2. Control send timer (100Hz): High-frequency command transmission
  
  // Action update timer (50Hz for neural network inference)
  action_update_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(action_update_period_)
    )),
    std::bind(&TrackTestNode::action_update_callback, this));
  
  // Control send timer (100Hz for sending control commands)
  control_send_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_send_period_)
    )),
    std::bind(&TrackTestNode::control_send_callback, this));
}

std::string TrackTestNode::get_px4_namespace(int drone_id)
{
  if (drone_id == 0) {
    return "/fmu/";
  } else {
    return "/px4_" + std::to_string(drone_id) + "/fmu/";
  }
}

void TrackTestNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
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

  // Check if ready to send TRAJ command (after 2.0s delay)
  if (waiting_hover_ && !hover_started_ && !hover_completed_ &&
      (this->now() - hover_detect_time_).seconds() > 2.0) {
    
    // 对于话题模式，需要等待目标数据就绪
    if (use_target_topic_) {
      if (target_generator_->is_target_ready()) {
        // Target data ready, send TRAJ command
        RCLCPP_INFO(this->get_logger(), 
                    "Target topic data ready, sending TRAJ state command");
        
        // Get target position information for logging
        target_generator_->get_target_position(target_x_, target_y_, target_z_);
        target_generator_->get_target_velocity(target_vx_, target_vy_, target_vz_);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Target position: [%.2f, %.2f, %.2f], velocity: [%.2f, %.2f, %.2f]",
                    target_x_, target_y_, target_z_, target_vx_, target_vy_, target_vz_);
        
        send_state_command(static_cast<int>(FsmState::TRAJ));
        hover_command_sent_ = true;
        hover_start_time_ = this->now();
      } else {
        // Target data not ready, print waiting message
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for target position topic data: %s (target data not ready, will not enter TRAJ state)",
                             target_position_topic_.c_str());
      }
    } else {
      // Static mode, send TRAJ command directly
      RCLCPP_INFO(this->get_logger(), 
                  "Static mode - sending TRAJ state command for hover control");
      send_state_command(static_cast<int>(FsmState::TRAJ));
      hover_command_sent_ = true;
      hover_start_time_ = this->now();
    }
  }

  // TRAJ detected - capture current position but WAIT before starting neural control
  if (state == FsmState::TRAJ && (waiting_hover_ || hover_command_sent_) && !hover_started_) {
    hover_started_ = true;
    waiting_hover_ = false;
    hover_command_sent_ = false;
    neural_control_ready_ = false;  // Reset - will be set to true after stabilization delay
    hover_start_time_ = this->now();
    step_counter_ = 0;  // Reset step counter
    
    // Capture current position for hover
    hover_x_ = current_x_;
    hover_y_ = current_y_;
    hover_z_ = current_z_;
    hover_yaw_ = current_yaw_;  // 使用当前yaw角
    
    // Initialize target using target generator
    if (!use_target_topic_) {
      // 静态模式: 在无人机正前方生成目标
      target_generator_->initialize_static_target(
        current_x_, current_y_, current_z_, current_yaw_, target_offset_distance_);
      
      // 获取生成的目标位置
      target_generator_->get_target_position(target_x_, target_y_, target_z_);
      target_generator_->get_target_velocity(target_vx_, target_vy_, target_vz_);
    } else {
      // Topic mode: target position was already obtained before sending TRAJ command, just confirm here
      if (!target_generator_->is_target_ready()) {
        RCLCPP_ERROR(this->get_logger(), 
                     "ERROR: Entered TRAJ state but target topic data is not ready! This should not happen.");
      } else {
        // Get latest target position again (may have been updated since last retrieval)
        target_generator_->get_target_position(target_x_, target_y_, target_z_);
        target_generator_->get_target_velocity(target_vx_, target_vy_, target_vz_);
        RCLCPP_INFO(this->get_logger(), 
                    "Target topic data confirmed ready");
      }
    }
    
    // Set initial hover action (will be used during stabilization period)
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      current_action_[0] = 2.0f * hover_thrust_ - 1.0f;  // Map [0,1] to [-1,1]
      current_action_[1] = 0.0f;
      current_action_[2] = 0.0f;
      current_action_[3] = 0.0f;
    }
    
    RCLCPP_INFO(this->get_logger(),
                "🚁 FSM entered TRAJ - Waiting %.2f s for mode stabilization...", 
                mode_stabilization_delay_);
    RCLCPP_INFO(this->get_logger(),
                "   Drone pos: [%.2f, %.2f, %.2f] yaw: %.2f rad | Target: [%.2f, %.2f, %.2f]",
                hover_x_, hover_y_, hover_z_, hover_yaw_, target_x_, target_y_, target_z_);
  }

  // Exited TRAJ, reset
  if (hover_started_ && state != FsmState::TRAJ) {
    hover_started_ = false;
    waiting_hover_ = false;
    hover_command_sent_ = false;
    neural_control_ready_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Left TRAJ state, resetting");
  }
}

void TrackTestNode::odom_callback(
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
void TrackTestNode::vehicle_rates_setpoint_callback(
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

// Action update callback (50Hz): Neural network inference
void TrackTestNode::action_update_callback()
{
  if (!odom_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for odometry...");
    return;
  }
  
  // Only run in TRAJ state
  if (current_state_ == FsmState::TRAJ && hover_started_ && !hover_completed_) {
    // Calculate elapsed time from timestamp
    double elapsed = (this->now() - hover_start_time_).seconds();
    
    // Update target position from target generator
    if (use_target_topic_) {
      // Topic mode: get latest target position from target_generator
      if (target_generator_->is_target_ready()) {
        target_generator_->get_target_position(target_x_, target_y_, target_z_);
        target_generator_->get_target_velocity(target_vx_, target_vy_, target_vz_);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for target position topic data...");
      }
    }
    // Static mode: target position was set during initialization and remains unchanged
    
    // Check if TRAJ control duration is complete
    if (elapsed >= hover_duration_) {
      RCLCPP_INFO(this->get_logger(), 
                  "✅ TRAJ control complete (%.1f s) - sending END_TRAJ command", hover_duration_);
      send_state_command(static_cast<int>(FsmState::END_TRAJ));
      hover_completed_ = true;
      return;
    }
    
    // Check if we need to wait for mode stabilization
    if (!neural_control_ready_ && elapsed < mode_stabilization_delay_) {
      // Still in stabilization period - send safe hover action
      std::lock_guard<std::mutex> lock(action_mutex_);
      current_action_[0] = 2.0f * hover_thrust_ - 1.0f;  // Map [0,1] to [-1,1]
      current_action_[1] = 0.0f;
      current_action_[2] = 0.0f;
      current_action_[3] = 0.0f;
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "⏳ Mode stabilization: %.2f/%.2f s - sending hover thrust",
                           elapsed, mode_stabilization_delay_);
      return;
    }
    
    // Initialize neural control after stabilization delay
    if (!neural_control_ready_ && elapsed >= mode_stabilization_delay_) {
      if (use_neural_control_ && policy_ && local_position_ready_ && attitude_ready_) {
        RCLCPP_INFO(this->get_logger(),
                    "✅ Mode stabilized (%.2f s) - Initializing neural control...",
                    elapsed);
        
        // Initialize buffer with zero observation vector
        std::vector<float> initial_obs_for_reset(OBS_DIM, 0.0f);  // Zero observation vector for buffer initialization
        
        // Hovering action (normalized): [thrust, omega_x=0, omega_y=0, omega_z=0]
        // Map hover_thrust_ from [0,1] to [-1,1] range for neural network input
        float hovering_thrust_normalized = 2.0f * hover_thrust_ - 1.0f;
        std::vector<float> hovering_action = {hovering_thrust_normalized, 0.0f, 0.0f, 0.0f};
        
        policy_->reset(initial_obs_for_reset, hovering_action);
        
        // // Print input buffer for step 0 (after reset, before first inference)
        // if (policy_) {
        //   std::vector<float> buffer_data = policy_->get_flattened_buffer();
        //   RCLCPP_INFO(this->get_logger(), "");
        //   RCLCPP_INFO(this->get_logger(), "========== STEP 0 INPUT BUFFER (650 dims) ==========");
        //   RCLCPP_INFO(this->get_logger(), "Buffer size: %zu", buffer_data.size());
          
        //   // Print buffer in format: [action(4), obs(9)] * 50
        //   constexpr int BUFFER_SIZE = 50;
        //   constexpr int ACTION_DIM = 4;
        //   constexpr int OBS_DIM = 9;
        //   for (int i = 0; i < BUFFER_SIZE; ++i) {
        //     int base_idx = i * (ACTION_DIM + OBS_DIM);
        //     RCLCPP_INFO(this->get_logger(), 
        //                 "  [Step %d] action=[%.6f, %.6f, %.6f, %.6f], obs=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
        //                 i,
        //                 buffer_data[base_idx + 0], buffer_data[base_idx + 1], 
        //                 buffer_data[base_idx + 2], buffer_data[base_idx + 3],  // action
        //                 buffer_data[base_idx + 4], buffer_data[base_idx + 5], buffer_data[base_idx + 6],  // obs[0:3]
        //                 buffer_data[base_idx + 7], buffer_data[base_idx + 8], buffer_data[base_idx + 9],  // obs[3:6]
        //                 buffer_data[base_idx + 10], buffer_data[base_idx + 11], buffer_data[base_idx + 12]); // obs[6:9]
        //   }
          
        //   // Print flattened buffer as single line (for easy copy-paste)
        //   RCLCPP_INFO(this->get_logger(), "Flattened buffer (single line, 650 values):");
        //   std::string buffer_str = "  [";
        //   for (size_t i = 0; i < buffer_data.size(); ++i) {
        //     char num_str[32];
        //     snprintf(num_str, sizeof(num_str), "%.6f", buffer_data[i]);
        //     buffer_str += num_str;
        //     if (i < buffer_data.size() - 1) {
        //       buffer_str += ", ";
        //     }
        //   }
        //   buffer_str += "]";
        //   // Split long string into multiple log lines for readability
        //   const size_t max_line_length = 200;
        //   size_t pos = 0;
        //   while (pos < buffer_str.length()) {
        //     size_t end_pos = std::min(pos + max_line_length, buffer_str.length());
        //     std::string line = buffer_str.substr(pos, end_pos - pos);
        //     RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
        //     pos = end_pos;
        //   }
        //   RCLCPP_INFO(this->get_logger(), "===================================================");
        // }
        
        // Get ACTUAL observation for first inference
        std::vector<float> obs_raw = get_observation();
        
        // Make a copy for normalization
        std::vector<float> obs_normalized = obs_raw;
        normalize_observation(obs_normalized);
        
        // Immediately run inference to get first action using normalized observation
        std::vector<float> first_action = policy_->get_action(obs_normalized);
        
        // Update current_action_ immediately (thread-safe)
        {
          std::lock_guard<std::mutex> lock(action_mutex_);
          current_action_ = first_action;
        }
        
        // Print first action details (same format as update_neural_action)
        double elapsed = (this->now() - hover_start_time_).seconds();
        
        // // ==================== 完整打印：步序号、原始观测、网络输出、归一化输出 ====================
        
        // // 1. 步序号 (this is step 1, the first inference after reset)
        // RCLCPP_INFO(this->get_logger(), "");  // 空行分隔
        // RCLCPP_INFO(this->get_logger(), "========== STEP 0 (t=%.3fs) ==========", elapsed);
        
        // // 2. 原始观测（未归一化）
        // RCLCPP_INFO(this->get_logger(), "[RAW OBS] v_body=[%.6f, %.6f, %.6f], g_body=[%.6f, %.6f, %.6f], target_pos_body=[%.6f, %.6f, %.6f]",
        //             obs_raw[0], obs_raw[1], obs_raw[2],    // v_body
        //             obs_raw[3], obs_raw[4], obs_raw[5],    // g_body
        //             obs_raw[6], obs_raw[7], obs_raw[8]);   // target_pos_body
        
        // // 3. 归一化后的观测（输入给神经网络的）
        // RCLCPP_INFO(this->get_logger(), "[NORM OBS] v_body=[%.6f, %.6f, %.6f], g_body=[%.6f, %.6f, %.6f], target_pos_body=[%.6f, %.6f, %.6f]",
        //             obs_normalized[0], obs_normalized[1], obs_normalized[2],    // v_body
        //             obs_normalized[3], obs_normalized[4], obs_normalized[5],    // g_body
        //             obs_normalized[6], obs_normalized[7], obs_normalized[8]);   // target_pos_body
        
        // // 4. 网络原始输出（[-1, 1]范围的tanh输出）
        // float thrust_raw = first_action[0];
        // float omega_x_norm = first_action[1];
        // float omega_y_norm = first_action[2];
        // float omega_z_norm = first_action[3];
        
        // RCLCPP_INFO(this->get_logger(), "[NN RAW OUTPUT] thrust_raw=%.6f, omega_x=%.6f, omega_y=%.6f, omega_z=%.6f",
        //             thrust_raw, omega_x_norm, omega_y_norm, omega_z_norm);
        
        // // 5. 归一化后输出（denormalized到物理量）
        // constexpr float OMEGA_MAX_X = 0.5f;  // rad/s
        // constexpr float OMEGA_MAX_Y = 0.5f;  // rad/s
        // constexpr float OMEGA_MAX_Z = 0.5f;  // rad/s
        // float roll_rate = omega_x_norm * OMEGA_MAX_X;
        // float pitch_rate = omega_y_norm * OMEGA_MAX_Y;
        // float yaw_rate = omega_z_norm * OMEGA_MAX_Z;
        // float thrust_normalized = (thrust_raw + 1.0f) * 0.5f;  // Map [-1,1] to [0,1]
        
        // RCLCPP_INFO(this->get_logger(), "[DENORM OUTPUT] thrust=[0-1]:%.6f, roll_rate=%.6f rad/s, pitch_rate=%.6f rad/s, yaw_rate=%.6f rad/s",
        //             thrust_normalized, roll_rate, pitch_rate, yaw_rate);
        
        // RCLCPP_INFO(this->get_logger(), "=====================================");
        
        neural_control_ready_ = true;
        RCLCPP_INFO(this->get_logger(), 
                    "🚀 Neural control active! (effective duration: %.1f s)",
                    hover_duration_ - elapsed);
      } else {
        // Fallback mode
        neural_control_ready_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Neural control not available - using fallback hover");
      }
      return;
    }
    
    // Neural control is ready - update action from neural network or use fallback
    if (neural_control_ready_) {
      if (use_neural_control_ && local_position_ready_ && attitude_ready_) {
        update_neural_action();
        
        // Log current status (throttled)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Neural control | pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] | target=[%.2f,%.2f,%.2f] | elapsed: %.1f/%.1f s",
                             current_x_, current_y_, current_z_,
                             current_vx_, current_vy_, current_vz_,
                             target_x_, target_y_, target_z_,
                             elapsed, hover_duration_);
      } else {
        // Fallback: Set hover action
        std::lock_guard<std::mutex> lock(action_mutex_);
        // Hover action: [hover_thrust in [-1,1] range, zero angular rates]
        current_action_[0] = 2.0f * hover_thrust_ - 1.0f;  // Map [0,1] to [-1,1]
        current_action_[1] = 0.0f;
        current_action_[2] = 0.0f;
        current_action_[3] = 0.0f;
        
        // Log current status (throttled)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Fallback hover control: thrust=%.3f | target=[%.2f,%.2f,%.2f] | elapsed: %.1f/%.1f s",
                             hover_thrust_, target_x_, target_y_, target_z_, elapsed, hover_duration_);
      }
    }
  }
}

// Control send callback (100Hz): High-frequency command transmission
void TrackTestNode::control_send_callback()
{
  if (!odom_ready_) {
    return;
  }
  
  // Only run in TRAJ state
  if (current_state_ == FsmState::TRAJ && hover_started_ && !hover_completed_) {
    // Publish current action at high frequency
    publish_current_action();
  }
}

// Get observation vector for neural network (9D)
// Based on TrackEnvVer6 observation space - all in body frame
// Observation composition:
// 1. 机体系速度 (3) - quad velocity in body frame
// 2. 机体系重力方向 (3) - gravity direction in body frame
// 3. 机体系目标位置 (3) - target relative position in body frame
std::vector<float> TrackTestNode::get_observation()
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

// Update neural network action (called at 50Hz)
void TrackTestNode::update_neural_action()
{
  // Increment step counter
  step_counter_++;
  
  // Update target position from target generator (for topic mode)
  // This ensures we always use the latest target position for observation calculation
  if (use_target_topic_ && target_generator_->is_target_ready()) {
    target_generator_->get_target_position(target_x_, target_y_, target_z_);
    target_generator_->get_target_velocity(target_vx_, target_vy_, target_vz_);
  }
  
  // Get current observation (9D) - RAW (before normalization)
  std::vector<float> obs_raw = get_observation();
  
  // Make a copy for normalization
  std::vector<float> obs_normalized = obs_raw;
  
  // Normalize observation to [-1, 1] range (CRITICAL: must match training!)
  normalize_observation(obs_normalized);
  
  // Get action from neural network
  // Note: With separate 50Hz timer, we don't need internal action repeat anymore
  std::vector<float> action = policy_->get_action(obs_normalized);
  
  // Update current action (thread-safe)
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    current_action_ = action;
  }
  
  // Calculate elapsed time
  double elapsed = (this->now() - hover_start_time_).seconds();
  
  // // ==================== 完整打印：步序号、原始观测、网络输出、归一化输出 ====================
  // Use throttled logging to reduce print frequency (every 2 seconds)
  
  // 1. 步序号
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "");  // 空行分隔
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                       "========== STEP %d (t=%.3fs) ==========", step_counter_, elapsed);
  
  // 2. 原始观测（未归一化）
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "[RAW OBS] v_body=[%.6f, %.6f, %.6f], g_body=[%.6f, %.6f, %.6f], target_pos_body=[%.6f, %.6f, %.6f]",
                       obs_raw[0], obs_raw[1], obs_raw[2],    // v_body
                       obs_raw[3], obs_raw[4], obs_raw[5],    // g_body
                       obs_raw[6], obs_raw[7], obs_raw[8]);   // target_pos_body
  
  // // 3. 归一化后的观测（输入给神经网络的）
  // RCLCPP_INFO(this->get_logger(), "[NORM OBS] v_body=[%.6f, %.6f, %.6f], g_body=[%.6f, %.6f, %.6f], target_pos_body=[%.6f, %.6f, %.6f]",
  //             obs_normalized[0], obs_normalized[1], obs_normalized[2],    // v_body
  //             obs_normalized[3], obs_normalized[4], obs_normalized[5],    // g_body
  //             obs_normalized[6], obs_normalized[7], obs_normalized[8]);   // target_pos_body
  
  // 4. 网络原始输出（[-1, 1]范围的tanh输出）
  float thrust_raw = action[0];
  float omega_x_norm = action[1];
  float omega_y_norm = action[2];
  float omega_z_norm = action[3];
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "[NN RAW OUTPUT] thrust_raw=%.6f, omega_x=%.6f, omega_y=%.6f, omega_z=%.6f",
                       thrust_raw, omega_x_norm, omega_y_norm, omega_z_norm);
  
  // // 5. 归一化后输出（denormalized到物理量）
  // constexpr float OMEGA_MAX_X = 0.5f;  // rad/s
  // constexpr float OMEGA_MAX_Y = 0.5f;  // rad/s
  // constexpr float OMEGA_MAX_Z = 0.5f;  // rad/s
  // float roll_rate = omega_x_norm * OMEGA_MAX_X;
  // float pitch_rate = omega_y_norm * OMEGA_MAX_Y;
  // float yaw_rate = omega_z_norm * OMEGA_MAX_Z;
  // float thrust_normalized = (thrust_raw + 1.0f) * 0.5f;  // Map [-1,1] to [0,1]
  
  // RCLCPP_INFO(this->get_logger(), "[DENORM OUTPUT] thrust=[0-1]:%.6f, roll_rate=%.6f rad/s, pitch_rate=%.6f rad/s, yaw_rate=%.6f rad/s",
  //             thrust_normalized, roll_rate, pitch_rate, yaw_rate);
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "=====================================");
}

// Publish current action (called at 100Hz)
void TrackTestNode::publish_current_action()
{
  // Read current action (thread-safe)
  std::vector<float> action;
  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action = current_action_;
  }
  
  // Create body rate setpoint message
  px4_msgs::msg::VehicleRatesSetpoint msg;
  
  // Action output: [thrust, omega_x, omega_y, omega_z]
  // All in range [-1, 1] from tanh activation
  float thrust_raw = action[0];
  float omega_x_norm = action[1];
  float omega_y_norm = action[2];
  float omega_z_norm = action[3];
  
  // Denormalize angular rates: [-1, 1] -> [-omega_max, omega_max]
  constexpr float OMEGA_MAX_X = 0.5f;  // Roll rate max [rad/s]
  constexpr float OMEGA_MAX_Y = 0.5f;  // Pitch rate max [rad/s]
  constexpr float OMEGA_MAX_Z = 0.5f;  // Yaw rate max [rad/s]
  
  msg.roll = omega_x_norm * OMEGA_MAX_X;
  msg.pitch = omega_y_norm * OMEGA_MAX_Y;
  msg.yaw = omega_z_norm * OMEGA_MAX_Z;
  
  // Thrust mapping: [-1, 1] -> [0, 1]
  float thrust_normalized = (thrust_raw + 1.0f) * 0.5f;
  msg.thrust_body[0] = 0.0f;
  msg.thrust_body[1] = 0.0f;
  msg.thrust_body[2] = -thrust_normalized;  // Negative for upward thrust in NED
  
  // Timestamp
  msg.timestamp = offboard_utils::get_timestamp_us(this->get_clock());
  
  // Calculate elapsed time from timestamp
  double elapsed = (this->now() - hover_start_time_).seconds();
  
  // Debug: Print control commands in the first 0.5 seconds
  if (elapsed < 0.5) {
    RCLCPP_INFO(this->get_logger(),
                "[t=%.3fs] [PUBLISH@100Hz] thrust=%.3f(raw=%.3f), rates=[%.3f, %.3f, %.3f] rad/s, "
                "thrust_body=[%.3f, %.3f, %.3f]",
                elapsed,
                thrust_normalized, thrust_raw,
                msg.roll, msg.pitch, msg.yaw,
                msg.thrust_body[0], msg.thrust_body[1], msg.thrust_body[2]);
  }
  
  // Publish
  rates_pub_->publish(msg);
}

// Publish hover setpoint using body rate control (zero angular velocity + hover thrust)
void TrackTestNode::publish_hover_setpoint()
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
  msg.timestamp = offboard_utils::get_timestamp_us(this->get_clock());
  // msg.timestamp = 0;
  
  // Publish
  rates_pub_->publish(msg);
  
  // Calculate elapsed time from timestamp
  double elapsed = (this->now() - hover_start_time_).seconds();
  
  // Debug: Print every frame in the first 0.5 seconds
  if (elapsed < 0.5) {
    RCLCPP_INFO(this->get_logger(),
                "[t=%.3fs] FALLBACK thrust=%.3f, rates=[0.0, 0.0, 0.0] rad/s",
                elapsed, hover_thrust_);
  }
}

void TrackTestNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}