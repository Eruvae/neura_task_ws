#include <cstdio>
#include "neura_task/neura_task_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeuraTaskNode>());
  rclcpp::shutdown();
  return 0;
}
