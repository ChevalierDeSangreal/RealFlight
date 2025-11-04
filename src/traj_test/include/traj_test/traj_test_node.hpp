#ifndef TRAJ_TEST_NODE_HPP_
#define TRAJ_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>
#include <string>

class TrajTestNode : public rclcpp::Node
{
public:
  explicit TrajTestNode(int drone_id);

private:
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  
  void generate_circular_trajectory(double t);
  void publish_trajectory_setpoint(double x, double y, double z, 
                                   double vx, double vy, double vz,
                                   double yaw);
  void send_state_command(int state);
  
  std::string get_px4_namespace(int drone_id);
  double calculate_effective_duration();  // NEW: Calculate duration based on max_speed

  // Node parameters
  int drone_id_;
  double circle_radius_;
  double circle_duration_;
  double ramp_up_time_;
  double ramp_down_time_;
  double initial_x_;
  double initial_y_;
  double initial_z_;
  double timer_period_;
  double max_speed_;              // NEW: Maximum tangential speed [m/s]
  bool use_max_speed_;            // NEW: Whether to use max_speed limit
  double effective_duration_;     // NEW: Actual duration after speed limiting
  
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
  bool waiting_traj_;
  bool traj_command_sent_;   
  bool traj_started_;
  rclcpp::Time hover_detect_time_;
  rclcpp::Time traj_start_time_;
  
  // Current position (reordered to match initialization order)
  double current_x_;
  double current_y_;
  double current_z_;
  bool odom_ready_;
  
  // ROS2 interfaces
  std::string px4_namespace_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TRAJ_TEST_NODE_HPP_