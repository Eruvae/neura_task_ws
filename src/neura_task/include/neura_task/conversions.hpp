#pragma once

#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

static inline std::vector<geometry_msgs::msg::Pose> convert_pose_stamped_to_pose(const std::vector<geometry_msgs::msg::PoseStamped> &pose_stamped_vec)
{
  std::vector<geometry_msgs::msg::Pose> pose_vec;
  pose_vec.reserve(pose_stamped_vec.size());
  for (const auto &pose_stamped : pose_stamped_vec)
  {
    pose_vec.push_back(pose_stamped.pose);
  }
  return pose_vec;
}