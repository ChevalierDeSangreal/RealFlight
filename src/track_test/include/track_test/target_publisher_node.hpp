#ifndef TARGET_PUBLISHER_NODE_HPP_
#define TARGET_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <string>

/**
 * @brief 目标物体位置发布节点 - 发布圆周运动轨迹的目标位置
 * 
 * 该节点生成圆周运动的目标轨迹，并发布目标的位置和速度信息，
 * 供追踪节点(track_test_node)订阅使用。
 */
class TargetPublisherNode : public rclcpp::Node
{
public:
  explicit TargetPublisherNode();

private:
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  
  /**
   * @brief 生成圆周运动轨迹
   * @param t 当前时间 (秒)
   */
  void generate_circular_trajectory(double t);
  
  /**
   * @brief 发布目标位置
   */
  void publish_target_position(double x, double y, double z);
  
  /**
   * @brief 发布目标速度
   */
  void publish_target_velocity(double vx, double vy, double vz);
  
  /**
   * @brief 计算指定时间的角度位置
   */
  double calculate_theta_at_time(double t);
  
  /**
   * @brief 计算指定时间的角速度
   */
  double calculate_angular_velocity_at_time(double t);
  
  /**
   * @brief 计算有效持续时间(考虑速度限制)
   */
  double calculate_effective_duration();

  // 圆周运动参数
  double circle_radius_;              // 圆形轨迹半径 [m]
  double circle_duration_;            // 单圈持续时间 [s]
  double circle_init_phase_;          // 初始相位角 [rad]
  int circle_times_;                  // 飞行圈数
  double ramp_up_time_;               // 加速时间 [s]
  double ramp_down_time_;             // 减速时间 [s]
  double stationary_time_;            // 初始静止时间 [s]
  
  // 圆心位置 (NED坐标系)
  double circle_center_x_;            // 圆心北向位置 [m]
  double circle_center_y_;            // 圆心东向位置 [m]
  double circle_center_z_;            // 圆心下向位置 [m] (负值表示高度)
  
  // 控制参数
  double timer_period_;               // 控制周期 [s]
  double max_speed_;                  // 最大线速度限制 [m/s] (-1 = 无限制)
  bool use_max_speed_;
  
  // 计算得到的参数
  double effective_duration_;         // 考虑速度限制后的实际持续时间 [s]
  double total_constant_duration_;    // 匀速阶段总时间 [s]
  double max_angular_vel_;            // 最大角速度 [rad/s]
  double angular_acceleration_;       // 角加速度 [rad/s²]
  
  // 状态机状态枚举（与track_test_node保持一致）
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
  
  // 运行状态
  bool trajectory_started_;
  rclcpp::Time start_time_;  // ROS时钟开始时间（用于sim_time模式）
  std::chrono::steady_clock::time_point start_time_system_;  // 系统时钟开始时间（用于onboard模式）
  bool use_sim_time_;  // 是否使用模拟时间
  
  // 状态机交互
  int drone_id_;                    // 无人机ID，用于确定状态话题
  FsmState current_state_;          // 当前状态机状态
  bool in_traj_state_;              // 是否已进入TRAJ状态
  
  // ROS2 接口
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;  // 状态订阅
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TARGET_PUBLISHER_NODE_HPP_

