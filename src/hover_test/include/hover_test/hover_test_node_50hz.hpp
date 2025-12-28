#ifndef HOVER_TEST_NODE_50HZ_HPP_
#define HOVER_TEST_NODE_50HZ_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "offboard_state_machine/utils.hpp"
#include "hover_test/tflite_policy_50hz.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <memory>
#include <deque>
#include <array>
#include <mutex>
#include <vector>

class HoverTestNode50Hz : public rclcpp::Node
{
public:
  explicit HoverTestNode50Hz(int drone_id);

private:
  // Timer callbacks (two-timer architecture)
  void action_update_callback();  // 50Hz: Neural network inference
  void control_send_callback();   // 100Hz: High-frequency command transmission
  
  // ROS2 subscriber callbacks
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void vehicle_rates_setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg);
  
  // Control functions
  void update_neural_action();       // Update action from neural network
  void publish_current_action();     // Publish current action at high frequency
  void publish_hover_setpoint();     // Fallback hover control
  void send_state_command(int state);
  bool check_safety_threshold();     // Check if distance to target exceeds safety threshold
  
  std::vector<float> get_observation();
  
  std::string get_px4_namespace(int drone_id);
  
  // Neural network dimensions (matching HoverEnv)
  static constexpr int OBS_DIM = 9;           // 机体系速度(3) + 机体系重力(3) + 机体系到目标点距离(3)

  // Basic parameters
  int drone_id_;
  double hover_duration_;             // Duration to hover [s]
  double hover_thrust_;               // Hover thrust [0.0-1.0]
  double mode_stabilization_delay_;   // Delay before starting neural control to ensure mode switch [s]
  
  // Control parameters (two-timer architecture)
  double action_update_period_;       // Action update period [s] (50Hz for NN inference)
  double control_send_period_;        // Control send period [s] (100Hz for command transmission)
  
  // State management
  enum class FsmState {
    INIT = 0,
    ARMING = 1,
    TAKEOFF = 2,
    GOTO = 3,
    HOVER = 4,
    TRAJ = 5,
    END_TRAJ = 6,
    LAND = 7,
    DONE = 8
  };
  
  FsmState current_state_;
  bool waiting_hover_;
  bool hover_command_sent_;
  bool hover_started_;
  bool hover_completed_;
  bool neural_control_ready_;         // True when mode stabilization delay has passed
  rclcpp::Time hover_detect_time_;
  rclcpp::Time hover_start_time_;
  
  // Hover position (captured when entering TRAJ state)
  double hover_x_;
  double hover_y_;
  double hover_z_;
  double hover_yaw_;
  
  // Current drone position (from odometry)
  double current_x_;
  double current_y_;
  double current_z_;
  bool odom_ready_;
  
  // Current drone state (for neural network observation)
  double current_vx_;
  double current_vy_;
  double current_vz_;
  double current_roll_;
  double current_pitch_;
  double current_yaw_;
  double current_roll_rate_;
  double current_pitch_rate_;
  double current_yaw_rate_;
  bool local_position_ready_;
  bool attitude_ready_;
  
  // Target point (目标点位置)
  double target_x_;
  double target_y_;
  double target_z_;
  
  // Safety thresholds
  double safety_threshold_horizontal_;  // Maximum horizontal distance to target [m]
  double safety_threshold_vertical_;    // Maximum vertical distance to target [m]
  
  // Neural network policy (50Hz version with buffer size 50)
  std::unique_ptr<TFLitePolicyInference50Hz> policy_;
  std::string model_path_;
  bool use_neural_control_;
  
  // Current action storage (shared between timers)
  std::vector<float> current_action_;  // [thrust, omega_x, omega_y, omega_z]
  std::mutex action_mutex_;            // Protect current_action_ from concurrent access
  
  // Step counter for debugging
  int step_counter_;                   // Track number of neural network inference steps
  
  // ROS2 interfaces
  std::string px4_namespace_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;  // Contains position, velocity, and attitude!
  
  // NOTE: These subscriptions are REDUNDANT (VehicleOdometry has all the data we need)
  // Kept in header for compatibility but not subscribed in implementation
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;  // REDUNDANT
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;        // REDUNDANT
  
  // Debug: Subscribe to PX4's actual vehicle rates setpoint (what PX4 is executing)
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_sub_;
  
  // Two-timer architecture
  rclcpp::TimerBase::SharedPtr action_update_timer_;  // 50Hz: Neural network inference
  rclcpp::TimerBase::SharedPtr control_send_timer_;   // 100Hz: Command transmission
};

#endif  // HOVER_TEST_NODE_50HZ_HPP_

