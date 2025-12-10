#include "hover_test/hover_test_node_50hz.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Get drone ID from command line or use default
  int drone_id = 0;
  if (argc > 1) {
    drone_id = std::atoi(argv[1]);
  }
  
  auto node = std::make_shared<HoverTestNode50Hz>(drone_id);
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting hover test node (50Hz) for drone %d", drone_id);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

