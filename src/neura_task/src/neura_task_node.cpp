#include <cstdio>
#include "neura_task/joint_command_publisher.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
