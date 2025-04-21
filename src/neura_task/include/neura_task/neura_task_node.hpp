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
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rviz_visual_tools/rviz_visual_tools.hpp"

using namespace std::chrono_literals;
using namespace std::string_view_literals;

class NeuraTaskNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandlePath = rclcpp_action::ClientGoalHandle<FollowPath>;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  enum class Task
  {
    TASK1,
    TASK2A,
    TASK2B
  };

  NeuraTaskNode()
  : Node("neura_task_node"), count_(0), task_(Task::TASK1)
  {
    declare_parameter("task", "task1");
    std::string task = get_parameter("task").as_string();

    // Task 1 parameters
    declare_parameter("circle_center_x", -1.0);
    declare_parameter("circle_center_y", 0.5);
    declare_parameter("circle_radius", 0.75);
    declare_parameter("circle_num_points", 16);
    declare_parameter("sine_joint_mid_values", std::vector<double>{M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0});
    declare_parameter("sine_joint_range", std::vector<double>{M_PI / 4.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0});
    declare_parameter("sine_joint_traj_duration", 10.0);
    declare_parameter("sine_joint_traj_steps", 100);

    // Task 2a parameters
    declare_parameter("task2a_start_pose", std::vector<double>{0.820, -0.375, 1.108, -0.460, 0.707, -0.458, 0.279});
    declare_parameter("task2a_end_pose", std::vector<double>{0.112, -0.701, 1.117 -0.009, 0.826, -0.563, -0.009});
    declare_parameter("task2a_linear_velocity", 0.1);
    declare_parameter("task2a_linear_acceleration", 0.1);
    declare_parameter("task2a_step_size", 0.01);

    // Task 2b parameters
    declare_parameter("task2b_base_move_type", "circle"); // use same parameters as task 1 for circle
    declare_parameter("task2b_endeffector_pose", std::vector<double>{-1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0});

    // line motion parameters
    declare_parameter("line_start_x", 0.0);
    declare_parameter("line_start_y", 0.5);
    declare_parameter("line_start_theta", 0.0);
    declare_parameter("line_end_x", 0.0);
    declare_parameter("line_end_y", 1.5);
    declare_parameter("line_end_theta", 0.0);

    using std::placeholders::_1;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/neura_task_markers", this));

    follow_joint_traj_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    waypoint_follower_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");
    path_follower_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    navigate_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

    if (!follow_joint_traj_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Follow joint trajectory action server not available after waiting");
      rclcpp::shutdown();
    }
    if (!waypoint_follower_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Follow waypoints action server not available after waiting");
      rclcpp::shutdown();
    }
    if (!path_follower_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Follow path action server not available after waiting");
      rclcpp::shutdown();
    }
    if (!navigate_to_pose_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Navigate to pose action server not available after waiting");
      rclcpp::shutdown();
    }
    if (!navigate_through_poses_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Navigate through poses action server not available after waiting");
      rclcpp::shutdown();
    }

    //main_loop_timer_ = this->create_wall_timer(1000ms, std::bind(&NeuraTaskNode::main_loop_callback, this));

    if (task == "task1") {
      RCLCPP_INFO(this->get_logger(), "Starting task 1");
      task_ = Task::TASK1;
      start_task1();
    } else if (task == "task2a") {
      task_ = Task::TASK2A;
      RCLCPP_INFO(this->get_logger(), "Starting task 2a");
      auto robot_description_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>("robot_description", robot_description_qos, std::bind(&NeuraTaskNode::robot_description_callback, this, _1));
    } else if (task == "task2b") {
      task_ = Task::TASK2B;
      RCLCPP_INFO(this->get_logger(), "Starting task 2b");
      auto robot_description_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>("robot_description", robot_description_qos, std::bind(&NeuraTaskNode::robot_description_callback, this, _1));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown task: %s", task.c_str());
    }

    /*std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    geometry_msgs::msg::PoseStamped center;
    center.pose.position.x = -1.0;
    center.pose.position.y = 0.5;
    center.pose.position.z = 0.0;
    center.pose.orientation.x = 0.0;
    center.pose.orientation.y = 0.0;
    center.pose.orientation.z = 0.0;
    center.pose.orientation.w = 1.0;
    waypoints = compute_circle_waypoint(center, 0.75, 16);
    send_nav2_waypoints(waypoints);*/

    //send_goal({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10);
    //rclcpp::sleep_for(std::chrono::seconds(10));
    //timer_ = this->create_wall_timer(
    //1000ms, std::bind(&NeuraTaskNode::timer_callback, this));
  }

  std::vector<geometry_msgs::msg::PoseStamped> compute_circle_waypoint(geometry_msgs::msg::PoseStamped &center, double radius, size_t num_points);

  void start_task1();

  void start_task2a();
  void start_task2b();

private:
  rclcpp::TimerBase::SharedPtr main_loop_timer_;
  rclcpp::TimerBase::SharedPtr task_retry_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr path_follower_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_;
  size_t count_;
  Task task_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos> ik_solver_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  const std::vector<std::string> joint_names = {"ur5eshoulder_pan_joint", "ur5eshoulder_lift_joint", "ur5eelbow_joint", "ur5ewrist_1_joint", "ur5ewrist_2_joint", "ur5ewrist_3_joint"};

  //static constexpr std::array<std::string_view, 6> joint_names = {"shoulder_pan_joint"sv, "shoulder_lift_joint"sv, "elbow_joint"sv, "wrist_1_joint"sv, "wrist_2_joint"sv, "wrist_3_joint"sv};

  void repeat_send_path(const std::vector<geometry_msgs::msg::PoseStamped> &path_points);
  void repeat_send_waypoints(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
  void repeat_send_joint_trajectory(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &joint_points);

  void compute_and_move_in_circle();
  void compute_and_move_in_line();

  void execute_computed_path(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &path);

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> test_joint_state(size_t num_joints);

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> compute_sine_joints(std::vector<double> mid_values, std::vector<double> range, double traj_duration, size_t steps);

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> compute_cartesian_path(geometry_msgs::msg::Pose &start, geometry_msgs::msg::Pose &end, double velocity, double acceleration, double step_size);

  void compute_and_move_to_cartesian_pose(const geometry_msgs::msg::Pose &pose);

  void main_loop_callback();

  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

  void test_solver();
};