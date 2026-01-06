#include "track_test/target_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TargetPublisherNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

