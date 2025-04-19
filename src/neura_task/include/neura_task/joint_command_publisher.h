#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using namespace std::chrono_literals;
using namespace std::string_view_literals;

class JointCommandPublisher : public rclcpp::Node
{
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandlePath = rclcpp_action::ClientGoalHandle<FollowPath>;

  public:
    JointCommandPublisher()
    : Node("joint_command_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);
      follow_joint_traj_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
      waypoint_follower_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");
      path_follower_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");

      std::vector<geometry_msgs::msg::PoseStamped> waypoints;
      geometry_msgs::msg::PoseStamped center;
      center.pose.position.x = -1.0;
      center.pose.position.y = 1.0;
      center.pose.position.z = 0.0;
      center.pose.orientation.x = 0.0;
      center.pose.orientation.y = 0.0;
      center.pose.orientation.z = 0.0;
      center.pose.orientation.w = 1.0;
      waypoints = compute_circle_waypoint(center, 0.5, 16);
      send_nav2_path(waypoints);

      send_goal({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10);
      //rclcpp::sleep_for(std::chrono::seconds(10));
      //timer_ = this->create_wall_timer(
      //1000ms, std::bind(&JointCommandPublisher::timer_callback, this));
    }

    std::vector<geometry_msgs::msg::PoseStamped> compute_circle_waypoint(geometry_msgs::msg::PoseStamped &center, double radius, size_t resoultion)
    {
      std::vector<geometry_msgs::msg::PoseStamped> res;
      res.resize(resoultion);
      for (size_t i=0; i < resoultion; i++)
      {
        double cur_frac = static_cast<double>(i) / static_cast<double>(resoultion-1);
        res[i].pose.position.x = center.pose.position.x + radius * cos(cur_frac * 2 * M_PI);
        res[i].pose.position.y = center.pose.position.y + radius * sin(cur_frac * 2 * M_PI);
        res[i].pose.position.z = center.pose.position.z;
        double yaw = atan2(res[i].pose.position.y - center.pose.position.y, res[i].pose.position.x - center.pose.position.x);
        res[i].pose.orientation.x = 0.0;
        res[i].pose.orientation.y = 0.0;
        res[i].pose.orientation.z = sin(yaw / 2);
        res[i].pose.orientation.w = cos(yaw / 2);
        res[i].header.frame_id = "map";
        res[i].header.stamp = this->now();
      }
      return res;
    }

  private:
    //static constexpr std::array<std::string_view, 6> joint_names = {"shoulder_pan_joint"sv, "shoulder_lift_joint"sv, "elbow_joint"sv, "wrist_1_joint"sv, "wrist_2_joint"sv, "wrist_3_joint"sv};

    void send_nav2_path(const std::vector<geometry_msgs::msg::PoseStamped> &goal_points)
    {
      using namespace std::placeholders;
  
      if (!this->path_follower_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }
  
      auto path_msg = FollowPath::Goal();
      path_msg.path.header.frame_id = "map";
      path_msg.path.header.stamp = this->now();
      path_msg.path.poses = goal_points;
  
      RCLCPP_INFO(this->get_logger(), "Sending waypoints");
  
      auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&JointCommandPublisher::path_goal_response_callback, this, _1);
      send_goal_options.feedback_callback = std::bind(&JointCommandPublisher::path_feedback_callback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&JointCommandPublisher::path_result_callback, this, _1);
      this->path_follower_client_->async_send_goal(path_msg, send_goal_options);
    }

    void path_goal_response_callback(const GoalHandlePath::SharedPtr &goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void path_feedback_callback(GoalHandlePath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
      /*std::stringstream ss;
      ss << "Next number in sequence received: ";
      for (auto number : feedback->partial_sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
    }

    void path_result_callback(const GoalHandlePath::WrappedResult &result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Goal reached");
      /*std::stringstream ss;
      ss << "Result received: ";
      for (auto number : result.result->sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
    }

    void send_nav2_waypoints(const std::vector<geometry_msgs::msg::PoseStamped> &goal_points)
    {
      using namespace std::placeholders;
  
      if (!this->waypoint_follower_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }
  
      auto waypoint_msg = FollowWaypoints::Goal();
      waypoint_msg.poses = goal_points;
  
      RCLCPP_INFO(this->get_logger(), "Sending waypoints");
  
      auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&JointCommandPublisher::waypoint_goal_response_callback, this, _1);
      send_goal_options.feedback_callback = std::bind(&JointCommandPublisher::waypoint_feedback_callback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&JointCommandPublisher::waypoint_result_callback, this, _1);
      this->waypoint_follower_client_->async_send_goal(waypoint_msg, send_goal_options);
    }

    void waypoint_goal_response_callback(const GoalHandleWaypoints::SharedPtr &goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void waypoint_feedback_callback(GoalHandleWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
      /*std::stringstream ss;
      ss << "Next number in sequence received: ";
      for (auto number : feedback->partial_sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
    }

    void waypoint_result_callback(const GoalHandleWaypoints::WrappedResult &result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Goal reached");
      /*std::stringstream ss;
      ss << "Result received: ";
      for (auto number : result.result->sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
    }

    void send_goal(const std::vector<double> &joint_values, double time_to_move)
    {
      using namespace std::placeholders;
  
      //this->timer_->cancel();
  
      if (!this->follow_joint_traj_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }
  
      auto goal_msg = FollowJointTrajectory::Goal();
      goal_msg.trajectory.joint_names = {"ur5eshoulder_pan_joint", "ur5eshoulder_lift_joint", "ur5eelbow_joint", "ur5ewrist_1_joint", "ur5ewrist_2_joint", "ur5ewrist_3_joint"};
      goal_msg.trajectory.points.resize(1);
      goal_msg.trajectory.points[0].positions = joint_values;
      goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(time_to_move);
  
      RCLCPP_INFO(this->get_logger(), "Sending goal");
  
      auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&JointCommandPublisher::joint_traj_goal_response_callback, this, _1);
      send_goal_options.feedback_callback = std::bind(&JointCommandPublisher::joint_traj_feedback_callback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&JointCommandPublisher::joint_traj_result_callback, this, _1);
      this->follow_joint_traj_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void joint_traj_goal_response_callback(const GoalHandleTrajectory::SharedPtr &goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

  void joint_traj_feedback_callback(GoalHandleTrajectory::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
  {
    /*std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
  }

  void joint_traj_result_callback(const GoalHandleTrajectory::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    /*std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
  }

    void move_to_joint_state(const std::vector<double> &joint_values, double time_to_move)
    {
      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = {"ur5eshoulder_pan_joint", "ur5eshoulder_lift_joint", "ur5eelbow_joint", "ur5ewrist_1_joint", "ur5ewrist_2_joint", "ur5ewrist_3_joint"};
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
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr path_follower_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_;
    size_t count_;
};