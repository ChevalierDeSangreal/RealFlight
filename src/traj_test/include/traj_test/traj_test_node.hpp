#ifndef TRAJ_TEST_NODE_HPP_
#define TRAJ_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "offboard_state_machine/utils.hpp"
#include "traj_test/tflite_policy.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <deque>
#include <array>

class TrajTestNode : public rclcpp::Node
{
public:
  explicit TrajTestNode(int drone_id);

private:
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void vehicle_rates_setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg);
  
  void publish_neural_control();
  void publish_hover_setpoint();
  void send_state_command(int state);
  
  std::vector<float> get_observation();
  
  std::string get_px4_namespace(int drone_id);
  
  // Neural network dimensions (matching TrackEnvVer6)
  static constexpr int OBS_DIM = 9;           // 机体系速度(3) + 机体系重力(3) + 机体系目标位置(3)

  // Basic parameters
  int drone_id_;
  double hover_duration_;             // Duration to hover [s]
  double hover_thrust_;               // Hover thrust [0.0-1.0]
  
  // Control parameters
  double timer_period_;               // Control loop period [s]
  
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
  rclcpp::Time hover_detect_time_;
  rclcpp::Time hover_start_time_;
  double accumulated_elapsed_;        // Accumulated time since hover start [s]
  
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
  
  // Target state (目标位置/轨迹)
  double target_x_;
  double target_y_;
  double target_z_;
  double target_vx_;
  double target_vy_;
  double target_vz_;
  
  // Neural network policy
  std::unique_ptr<TFLitePolicyInference> policy_;
  std::string model_path_;
  bool use_neural_control_;
  
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
  
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TRAJ_TEST_NODE_HPP_