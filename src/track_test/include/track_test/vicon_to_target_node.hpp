#ifndef VICON_TO_TARGET_NODE_HPP_
#define VICON_TO_TARGET_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <string>
#include <deque>

/**
 * @brief Vicon到Target转换节点 - 将Vicon话题转换为Target话题
 * 
 * 该节点订阅Vicon系统发布的位置信息（PoseStamped或TransformStamped），
 * 并将其转换为target位置和速度话题，供追踪节点(track_test_node)订阅使用。
 */
class ViconToTargetNode : public rclcpp::Node
{
public:
  explicit ViconToTargetNode();

private:
  /**
   * @brief Vicon PoseStamped 回调函数
   */
  void vicon_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  /**
   * @brief Vicon TransformStamped 回调函数
   */
  void vicon_transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  
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
  std::string vicon_topic_name_;        // Vicon话题名称
  std::string vicon_topic_type_;       // Vicon话题类型: "PoseStamped" 或 "TransformStamped"
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr vicon_transform_sub_;
};

#endif  // VICON_TO_TARGET_NODE_HPP_

