#include "track_test/vicon_to_target_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ViconToTargetNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

