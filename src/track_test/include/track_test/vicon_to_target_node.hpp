#ifndef VICON_TO_TARGET_NODE_HPP_
#define VICON_TO_TARGET_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>
#include <string>
#include <deque>

/**
 * @brief PX4到Target转换节点 - 将PX4 VehicleOdometry话题转换为Target话题
 * 
 * 该节点订阅PX4发布的VehicleOdometry消息，
 * 并将其转换为target位置和速度话题，供追踪节点(track_test_node)订阅使用。
 */
class ViconToTargetNode : public rclcpp::Node
{
public:
  explicit ViconToTargetNode();

private:
  /**
   * @brief PX4 VehicleOdometry 回调函数
   */
  void px4_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  
  /**
   * @brief 发布目标位置
   */
  void publish_target_position(double x, double y, double z);
  
  /**
   * @brief 发布目标速度（通过位置差分计算）
   */
  void publish_target_velocity(double vx, double vy, double vz);
  
  /**
   * @brief 从位置历史计算速度
   */
  void calculate_velocity_from_position(const geometry_msgs::msg::Point& current_pos,
                                       const rclcpp::Time& current_time);

  // 参数
  std::string px4_topic_name_;         // PX4 VehicleOdometry话题名称
  double velocity_calc_window_;        // 速度计算时间窗口 [s]
  int max_history_size_;               // 位置历史最大长度
  
  // 位置历史（用于速度计算）
  struct PositionHistory {
    geometry_msgs::msg::Point position;
    rclcpp::Time timestamp;
  };
  std::deque<PositionHistory> position_history_;
  
  // 当前状态
  bool has_position_data_;              // 是否已收到位置数据
  geometry_msgs::msg::Point last_position_;
  rclcpp::Time last_time_;
  
  // ROS2 接口
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
};

#endif  // VICON_TO_TARGET_NODE_HPP_

