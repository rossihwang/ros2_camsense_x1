// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <camsense_x1/camsense_x1.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamsenseX1>("camsense_x1_node", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  
  return 0;
}
