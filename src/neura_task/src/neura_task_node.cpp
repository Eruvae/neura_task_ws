#include "neura_task/neura_task_node.hpp"

std::vector<geometry_msgs::msg::PoseStamped> NeuraTaskNode::compute_circle_waypoint(geometry_msgs::msg::PoseStamped &center, double radius, size_t num_points)
{
  std::vector<geometry_msgs::msg::PoseStamped> res;
  res.resize(num_points);
  for (size_t i=0; i < num_points; i++)
  {
    //double cur_frac = static_cast<double>(i) / static_cast<double>(num_points-1);
    double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
    double x = center.pose.position.x + radius * cos(theta);
    double y = center.pose.position.y + radius * sin(theta);
    double yaw = theta + M_PI / 2.0;

    res[i].pose.position.x = center.pose.position.x + radius * cos(theta);
    res[i].pose.position.y = center.pose.position.y + radius * sin(theta);
    res[i].pose.position.z = 0.0;
    //double yaw = atan2(res[i].pose.position.y - center.pose.position.y, res[i].pose.position.x - center.pose.position.x);
    res[i].pose.orientation.x = 0.0;
    res[i].pose.orientation.y = 0.0;
    res[i].pose.orientation.z = sin(yaw / 2);
    res[i].pose.orientation.w = cos(yaw / 2);
    res[i].header.frame_id = "map";
    res[i].header.stamp = this->now();
  }
  return res;
}

void NeuraTaskNode::repeat_send_path(const std::vector<geometry_msgs::msg::PoseStamped> &path_points)
{
  using namespace std::placeholders;

  if (!this->path_follower_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto path_msg = FollowPath::Goal();
  path_msg.path.header.frame_id = "map";
  path_msg.path.header.stamp = this->now();
  path_msg.path.poses = path_points;

  RCLCPP_INFO(this->get_logger(), "Executing path");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Follow path was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Follow path accepted by server, waiting for result");
    }
  };
  send_goal_options.result_callback = [this, path_points](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Follow path failed: '%s'", result.result->error_msg.c_str());
      return;
    }

    // Execute the path again
    repeat_send_path(path_points);
  };
  path_follower_client_->async_send_goal(path_msg, send_goal_options);
}

void NeuraTaskNode::send_nav2_path(const std::vector<geometry_msgs::msg::PoseStamped> &goal_points)
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
  send_goal_options.goal_response_callback = std::bind(&NeuraTaskNode::path_goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&NeuraTaskNode::path_feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&NeuraTaskNode::path_result_callback, this, _1);
  this->path_follower_client_->async_send_goal(path_msg, send_goal_options);
}

void NeuraTaskNode::path_goal_response_callback(const GoalHandlePath::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Path was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Path accepted by server, waiting for result");
  }
}

void NeuraTaskNode::path_feedback_callback(GoalHandlePath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  /*std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
}

void NeuraTaskNode::path_result_callback(const GoalHandlePath::WrappedResult &result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Path was aborted: '%s'", result.result->error_msg.c_str());
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Path was canceled '%s'", result.result->error_msg.c_str());
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  if (result.result->error_code != 0) {
    RCLCPP_ERROR(this->get_logger(), "Path failed: '%s'", result.result->error_msg.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Path successfully executed: '%s'", result.result->error_msg.c_str());
  /*std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
}

void NeuraTaskNode::send_nav2_waypoints(const std::vector<geometry_msgs::msg::PoseStamped> &goal_points)
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
  send_goal_options.goal_response_callback = std::bind(&NeuraTaskNode::waypoint_goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&NeuraTaskNode::waypoint_feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&NeuraTaskNode::waypoint_result_callback, this, _1);
  this->waypoint_follower_client_->async_send_goal(waypoint_msg, send_goal_options);
}

void NeuraTaskNode::waypoint_goal_response_callback(const GoalHandleWaypoints::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void NeuraTaskNode::waypoint_feedback_callback(GoalHandleWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  /*std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
}

void NeuraTaskNode::waypoint_result_callback(const GoalHandleWaypoints::WrappedResult &result)
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

void NeuraTaskNode::send_goal(const std::vector<double> &joint_values, double time_to_move)
{
  using namespace std::placeholders;

  //this->timer_->cancel();

  if (!this->follow_joint_traj_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points.resize(1);
  goal_msg.trajectory.points[0].positions = joint_values;
  goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(time_to_move);

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&NeuraTaskNode::joint_traj_goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&NeuraTaskNode::joint_traj_feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&NeuraTaskNode::joint_traj_result_callback, this, _1);
  this->follow_joint_traj_client_->async_send_goal(goal_msg, send_goal_options);
}

void NeuraTaskNode::joint_traj_goal_response_callback(const GoalHandleTrajectory::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void NeuraTaskNode::joint_traj_feedback_callback(GoalHandleTrajectory::SharedPtr, const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
  /*std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());*/
}

void NeuraTaskNode::joint_traj_result_callback(const GoalHandleTrajectory::WrappedResult &result)
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

void NeuraTaskNode::move_to_joint_state(const std::vector<double> &joint_values, double time_to_move)
{
  auto message = trajectory_msgs::msg::JointTrajectory();
  message.joint_names = joint_names;
  message.points.resize(1);
  message.points[0].positions = joint_values;
  message.points[0].time_from_start = rclcpp::Duration::from_seconds(time_to_move);
  publisher_->publish(message);
}

std::vector<trajectory_msgs::msg::JointTrajectoryPoint> NeuraTaskNode::test_joint_state(size_t num_joints)
{
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res;
  res.resize(1);
  res[0].positions.resize(num_joints);
  //res[0].positions[0] = M_PI;
  res[0].time_from_start = rclcpp::Duration::from_seconds(5);
  return res;
}

// assume max/min ang around 0
std::vector<trajectory_msgs::msg::JointTrajectoryPoint> NeuraTaskNode::compute_sine_joints(std::vector<double> mid_values, std::vector<double> range, double traj_duration, double steps)
{
  if (mid_values.size() != range.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Mid values and range size mismatch");
    return {};
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res;
  size_t num_joints = mid_values.size();
  res.resize(steps);
  for (size_t i=0; i < steps; i++)
  {
    res[i].positions.resize(num_joints);
    double cur_frac = static_cast<double>(i) / static_cast<double>(steps-1);
    for (size_t j=0; j < num_joints; j++)
    {
      res[i].positions[j] = mid_values[j] + range[j] * sin(cur_frac * 2 * M_PI);
      //RCLCPP_INFO(this->get_logger(), "Frac: '%f', Pos: '%f'", cur_frac, res[i].positions[j]);
    }
    res[i].time_from_start = rclcpp::Duration::from_seconds(traj_duration * cur_frac);
  }
  return res;
}

void NeuraTaskNode::main_loop_callback()
{
  /*constexpr double MAX_ANG = M_PI;
  static size_t cur_step = 0;
  constexpr size_t MAX_STEP = 10;
  std::vector<double> joint_states(6);
  double cur_frac = static_cast<double>(cur_step+1) / static_cast<double>(MAX_STEP);
  joint_states[0] = MAX_ANG * sin(cur_frac * 2 * M_PI);
  move_to_joint_state(joint_states, 1);*/

  main_loop_timer_->cancel();
  /*double TRAJ_DURATION = 10.0;
  size_t STEPS = 100;
  auto message = trajectory_msgs::msg::JointTrajectory();
  message.joint_names = joint_names;
  message.points = compute_sine_joints(MAX_ANG, TRAJ_DURATION, STEPS, 6);
  //message.points = test_joint_state(6);
  //message.data = "Hello, world! " + std::to_string(count_++);
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);*/
}

void NeuraTaskNode::start_task1()
{
  // move robot to start position
  using namespace std::placeholders;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  geometry_msgs::msg::PoseStamped center;
  center.pose.position.x = -1.0;
  center.pose.position.y = 0.5;
  center.pose.position.z = 0.0;
  center.pose.orientation.x = 0.0;
  center.pose.orientation.y = 0.0;
  center.pose.orientation.z = 0.0;
  center.pose.orientation.w = 1.0;
  waypoints = compute_circle_waypoint(center, 0.75, 16);

  auto start_pose = waypoints[0];

  auto move_msg = NavigateToPose::Goal();
  move_msg.pose.header.frame_id = "map";
  move_msg.pose.header.stamp = this->now();
  move_msg.pose.pose = start_pose.pose;

  RCLCPP_INFO(this->get_logger(), "Move to start position");

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Start pose was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Start pose accepted by server, waiting for result");
    }
  };
  send_goal_options.result_callback = [this, waypoints](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Start pose navigation failed: '%s'", result.result->error_msg.c_str());
      return;
    }

    // Move on path along waypoints in circle
    repeat_send_path(waypoints);
  };

  this->navigate_to_pose_client_->async_send_goal(move_msg, send_goal_options);

  // move arm to start joint state (pi/2, -pi/2, 0, -pi/2, 0, 0)
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points.resize(1);
  joint_traj_msg.trajectory.points[0].positions = {M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0};
  double time_to_move = 5.0;
  joint_traj_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(time_to_move);

  RCLCPP_INFO(this->get_logger(), "Moving arm joints to start position");

  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Start joint configuration was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Start joint configuration accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Start joint configuration failed: '%s'", result.result->error_string.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Start joint configuration successfully executed: '%s'", result.result->error_string.c_str());

    // TODO Move in sinus motions on reaching start positions
    auto joint_traj_msg = FollowJointTrajectory::Goal();
    joint_traj_msg.trajectory.joint_names = joint_names;
    joint_traj_msg.trajectory.points = compute_sine_joints(
      {M_PI / 2.0, -M_PI / 2.0,        0.0, -M_PI / 2.0,        0.0,        0.0},
      {M_PI / 4.0,  M_PI / 4.0, M_PI / 4.0,  M_PI / 4.0, M_PI / 4.0, M_PI / 4.0},
      10.0, 100);

    auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);

    // TODO repeat arm motion
  };

  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);

}