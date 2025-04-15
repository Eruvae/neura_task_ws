#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;
using namespace std::string_view_literals;

class JointCommandPublisher : public rclcpp::Node
{
  public:
    JointCommandPublisher()
    : Node("joint_command_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);
      move_to_joint_state({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10);
      rclcpp::sleep_for(std::chrono::seconds(10));
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&JointCommandPublisher::timer_callback, this));
    }

  private:
    //static constexpr std::array<std::string_view, 6> joint_names = {"shoulder_pan_joint"sv, "shoulder_lift_joint"sv, "elbow_joint"sv, "wrist_1_joint"sv, "wrist_2_joint"sv, "wrist_3_joint"sv};

    void move_to_joint_state(const std::vector<double> &joint_values, double time_to_move)
    {
      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
      message.points.resize(1);
      message.points[0].positions = joint_values;
      message.points[0].time_from_start = rclcpp::Duration::from_seconds(time_to_move);
      publisher_->publish(message);
    }

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> test_joint_state(size_t num_joints)
    {
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res;
      res.resize(1);
      res[0].positions.resize(num_joints);
      //res[0].positions[0] = M_PI;
      res[0].time_from_start = rclcpp::Duration::from_seconds(5);
      return res;
    }

    // assume max/min ang around 0
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> compute_sine_joints(double max_ang, double traj_duration, double steps, size_t num_joints)
    {
      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res;
      res.resize(steps);
      for (size_t i=0; i < steps; i++)
      {
        res[i].positions.resize(num_joints);
        double cur_frac = static_cast<double>(i) / static_cast<double>(steps-1);
        res[i].positions[0] = max_ang * sin(cur_frac * 2 * M_PI);
        RCLCPP_INFO(this->get_logger(), "Frac: '%f', Pos: '%f'", cur_frac, res[i].positions[0]);
        res[i].time_from_start = rclcpp::Duration::from_seconds(traj_duration * cur_frac);
      }
      return res;
    }

    void timer_callback()
    {
      constexpr double MAX_ANG = M_PI;
      static size_t cur_step = 0;
      constexpr size_t MAX_STEP = 10;
      std::vector<double> joint_states(6);
      double cur_frac = static_cast<double>(cur_step+1) / static_cast<double>(MAX_STEP);
      joint_states[0] = MAX_ANG * sin(cur_frac * 2 * M_PI);
      move_to_joint_state(joint_states, 1);
      /*double TRAJ_DURATION = 10.0;
      size_t STEPS = 100;
      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
      message.points = compute_sine_joints(MAX_ANG, TRAJ_DURATION, STEPS, 6);
      //message.points = test_joint_state(6);
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);*/
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};