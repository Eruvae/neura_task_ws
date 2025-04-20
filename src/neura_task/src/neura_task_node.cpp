#include "neura_task/neura_task_node.hpp"

#include "kdl/rotational_interpolation_sa.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

std::vector<geometry_msgs::msg::PoseStamped> NeuraTaskNode::compute_circle_waypoint(geometry_msgs::msg::PoseStamped &center, double radius, size_t num_points)
{
  std::vector<geometry_msgs::msg::PoseStamped> res(num_points + 1);
  for (size_t i=0; i < num_points; i++)
  {
    //double cur_frac = static_cast<double>(i) / static_cast<double>(num_points-1);
    double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
    double x = center.pose.position.x + radius * cos(theta);
    double y = center.pose.position.y + radius * sin(theta);
    double yaw = theta + M_PI / 2.0;

    res[i].pose.position.x = x;
    res[i].pose.position.y = y;
    res[i].pose.position.z = 0.0;
    //double yaw = atan2(res[i].pose.position.y - center.pose.position.y, res[i].pose.position.x - center.pose.position.x);
    res[i].pose.orientation.x = 0.0;
    res[i].pose.orientation.y = 0.0;
    res[i].pose.orientation.z = sin(yaw / 2);
    res[i].pose.orientation.w = cos(yaw / 2);
    res[i].header.frame_id = "map";
    res[i].header.stamp = this->now();
  }
  // add start pose at the end to complete the circle
  res.back().pose = res[0].pose;
  return res;
}

void NeuraTaskNode::repeat_send_path(const std::vector<geometry_msgs::msg::PoseStamped> &path_points)
{
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

void NeuraTaskNode::repeat_send_joint_trajectory(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &joint_points)
{
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points = joint_points;

  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Joint trajectory was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Joint trajectory accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this, joint_points](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Joint trajectory failed: '%s'", result.result->error_string.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Joint trajectory successfully executed: '%s'", result.result->error_string.c_str());
    repeat_send_joint_trajectory(joint_points);
  };

  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);
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
std::vector<trajectory_msgs::msg::JointTrajectoryPoint> NeuraTaskNode::compute_sine_joints(std::vector<double> mid_values, std::vector<double> range, double traj_duration, size_t steps)
{
  if (mid_values.size() != range.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Mid values and range size mismatch");
    return {};
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res(steps);
  size_t num_joints = mid_values.size();
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

std::vector<trajectory_msgs::msg::JointTrajectoryPoint> NeuraTaskNode::compute_cartesian_path(geometry_msgs::msg::Pose &start, geometry_msgs::msg::Pose &end, double velocity, double acceleration, double step_size)
{
  KDL::JntArray init_positions(chain_.getNrOfJoints());
  KDL::Frame start_frame = KDL::Frame(KDL::Rotation::Quaternion(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w), KDL::Vector(start.position.x, start.position.y, start.position.z));
  KDL::Frame end_frame = KDL::Frame(KDL::Rotation::Quaternion(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w), KDL::Vector(end.position.x, end.position.y, end.position.z));
  KDL::JntArray result_positions(chain_.getNrOfJoints());
  if (ik_solver_->CartToJnt(init_positions, start_frame, result_positions) < 0) {
    RCLCPP_ERROR(this->get_logger(), "IK solver failed to compute start joint configuration");
    return {};
  }
  KDL::RotationalInterpolation_SingleAxis rot_interp;
  rot_interp.SetStartEnd(start_frame.M, end_frame.M);
  double path_length = sqrt(pow(end.position.x - start.position.x, 2) + pow(end.position.y - start.position.y, 2) + pow(end.position.z - start.position.z, 2));
  double t_accel = velocity / acceleration;
  double d_accel = 0.5 * acceleration * t_accel * t_accel;
  double total_time = 0.0;
  bool triangular = false;

  if (2.0 * d_accel >= path_length)
  {
    // triangular profile
    t_accel = std::sqrt(path_length / acceleration);
    total_time = 2.0 * t_accel;
    triangular = true;
  }
  else
  {
    double d_const = path_length - 2.0 * d_accel;
    double t_const = d_const / velocity;
    total_time = 2.0 * t_accel + t_const;
  }

  size_t num_steps = static_cast<size_t>(path_length / step_size);
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> res(num_steps);
  for (size_t i=0; i < num_steps; i++)
  {
    res[i].positions.resize(chain_.getNrOfJoints());
    double cur_frac = static_cast<double>(i) / static_cast<double>(num_steps-1);
    double s = cur_frac * path_length; // current distance along the path
    KDL::Rotation cur_rot = rot_interp.Pos(cur_frac * rot_interp.Angle());
    KDL::Vector cur_pos = start_frame.p + (end_frame.p - start_frame.p) * cur_frac;
    KDL::Frame cur_frame = KDL::Frame(cur_rot, cur_pos);
    if (ik_solver_->CartToJnt(init_positions, cur_frame, result_positions) < 0) {
      RCLCPP_ERROR(this->get_logger(), "IK solver failed to compute joint configuration, returning partial path (fraction: %f)", cur_frac);
      res.resize(i);
      return res;
    }
    for (size_t j=0; j < chain_.getNrOfJoints(); j++)
    {
      res[i].positions[j] = result_positions(j);
      //RCLCPP_INFO(this->get_logger(), "Frac: '%f', Pos: '%f'", cur_frac, res[i].positions[j]);
    }

    // compute time from start
    double t = 0.0;
    if (triangular) {
      if (s < 0.5 * path_length) {
        t = std::sqrt(2.0 * s / acceleration);
      }
      else {
        double s_remain = path_length - s;
        t = total_time - std::sqrt(2.0 * s_remain / acceleration);
      }
    }
    else {
      if (s < d_accel) {
        t = std::sqrt(2.0 * s / acceleration);
      }
      else if (s < (path_length - d_accel)) {
        double s_const = s - d_accel;
        t = t_accel + s_const / velocity;
      }
      else {
        double s_remain = path_length - s;
        t = total_time - std::sqrt(2.0 * s_remain / acceleration);
      }
    }

    res[i].time_from_start = rclcpp::Duration::from_seconds(t);
    init_positions = result_positions; // update initial positions for next step
  }
  return res;
}

void NeuraTaskNode::execute_computed_path(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &path)
{
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points = path;
  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Path was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Path accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Path execution failed: '%s'", result.result->error_string.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Path successfully executed: '%s'", result.result->error_string.c_str());
  };
  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);
}

void NeuraTaskNode::start_task2a()
{
  // TODO: dynamically get poses from ros params
  // TODO: transform poses to base_link frame
  // TODO: move base in best position to reach both poses

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.757;
  start_pose.position.y = 0.135;
  start_pose.position.z = 1.288;
  start_pose.orientation.x = 0.864;
  start_pose.orientation.y = 0.371;
  start_pose.orientation.z = -0.100;
  start_pose.orientation.w = -0.325;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = 0.106;
  end_pose.position.y = 0.258;
  end_pose.position.z = 1.763;
  end_pose.orientation.x = -0.655;
  end_pose.orientation.y = 0.268;
  end_pose.orientation.z = 0.133;
  end_pose.orientation.w = 0.693;

  // compute cartesian path
  auto cartesian_path = compute_cartesian_path(start_pose, end_pose, 0.1, 0.1, 0.01);
  if (cartesian_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute cartesian path");
    return;
  }

  // go to start pose
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points.resize(1);
  joint_traj_msg.trajectory.points[0].positions = cartesian_path[0].positions;
  joint_traj_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(5.0);
  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this, cartesian_path](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Start joint configuration was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Start joint configuration accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this, cartesian_path](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "Start joint configuration failed: '%s'", result.result->error_string.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Start joint configuration successfully executed: '%s'", result.result->error_string.c_str());

    // Execute cartesian path
    RCLCPP_INFO(this->get_logger(), "Executing cartesian path");
    execute_computed_path(cartesian_path);
  };
  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);
}

void NeuraTaskNode::compute_and_move_to_cartesian_pose(const geometry_msgs::msg::Pose &pose)
{
  task_retry_timer_->cancel();
  // compute current robot pose
  geometry_msgs::msg::TransformStamped base_link_transform;
  std::string to_frame = "base_link";
  std::string from_frame = "map";
  KDL::JntArray result_positions(chain_.getNrOfJoints());

  try {
    base_link_transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", to_frame.c_str(), from_frame.c_str(), ex.what());
    task_retry_timer_->reset();
    return;
  }

  geometry_msgs::msg::Pose pose_base_link;
  tf2::doTransform(pose, pose_base_link, base_link_transform);

  KDL::JntArray init_positions(chain_.getNrOfJoints());
  KDL::Frame frame = KDL::Frame(KDL::Rotation::Quaternion(pose_base_link.orientation.x, pose_base_link.orientation.y, pose_base_link.orientation.z, pose_base_link.orientation.w), KDL::Vector(pose_base_link.position.x, pose_base_link.position.y, pose_base_link.position.z));
  if (ik_solver_->CartToJnt(init_positions, frame, result_positions) < 0) {
    RCLCPP_ERROR(this->get_logger(), "IK solver failed to compute joint configuration, repeat trying");
    task_retry_timer_->reset();
    return;
  }
  
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points.resize(1);
  joint_traj_msg.trajectory.points[0].positions.resize(chain_.getNrOfJoints());
  for (size_t i=0; i < chain_.getNrOfJoints(); i++)
  {
    joint_traj_msg.trajectory.points[0].positions[i] = result_positions(i);
  }
  joint_traj_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0); // TODO: compute time from start based on distance to current position
  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "IK joint configuration was rejected by server");
      task_retry_timer_->reset();
    } else {
      RCLCPP_INFO(this->get_logger(), "IK joint configuration accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this, pose](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "IK joint configuration failed: '%s'", result.result->error_string.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "IK joint configuration successfully executed: '%s'", result.result->error_string.c_str());
    task_retry_timer_->reset();
  };
  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);
}

void NeuraTaskNode::start_task2b()
{
  // Move in the circle from task 1
  compute_and_move_in_circle();

  // Keep arm endeffector in the middle of the circle
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position.x = -1.0;
  middle_pose.position.y = 0.5;
  middle_pose.position.z = 1.0;
  middle_pose.orientation.x = 0.0;
  middle_pose.orientation.y = 0.0;
  middle_pose.orientation.z = 0.0;
  middle_pose.orientation.w = 1.0;

  task_retry_timer_ = create_wall_timer(100ms, [this, middle_pose]() {
    this->compute_and_move_to_cartesian_pose(middle_pose);
  });
}

void NeuraTaskNode::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Robot description: '%s'", msg->data.c_str());
  // Construct KDL tree from URDF
  const std::string urdf = msg->data;
  kdl_parser::treeFromString(urdf, tree_);
  
  RCLCPP_INFO(this->get_logger(), "Root: '%s', joints: '%u', segments: '%u'", tree_.getRootSegment()->first.c_str(), tree_.getNrOfJoints(), tree_.getNrOfSegments());

  // Get the chain from the root to the end effector
  if (!tree_.getChain("base_link", "ur5etool0", chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get chain from base_link to tool0");
    return;
  }

  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
  //test_solver();

  if (task_ == Task::TASK2A) {
    start_task2a();
  }
  else if (task_ == Task::TASK2B) {
    start_task2b();
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Unknown task");
  }
}

void NeuraTaskNode::test_solver()
{
  // Test solver
  KDL::JntArray init_positions(chain_.getNrOfJoints());
  const KDL::Frame pose = KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0.5, 0.5, 0.5));
  KDL::JntArray result_positions(chain_.getNrOfJoints());
  if (ik_solver_->CartToJnt(init_positions, pose, result_positions) < 0) {
    RCLCPP_ERROR(this->get_logger(), "IK solver failed");
    return;
  }
  Eigen::IOFormat joint_print_fmt(Eigen::StreamPrecision, 0, ", ", " ", "[", "]");
  RCLCPP_INFO_STREAM(this->get_logger(), "IK solver result: " << result_positions.data.format(joint_print_fmt));

  // Execute result
  auto joint_traj_msg = FollowJointTrajectory::Goal();
  joint_traj_msg.trajectory.joint_names = joint_names;
  joint_traj_msg.trajectory.points.resize(1);
  joint_traj_msg.trajectory.points[0].positions.resize(chain_.getNrOfJoints());
  for (size_t i=0; i < chain_.getNrOfJoints(); i++)
  {
    joint_traj_msg.trajectory.points[0].positions[i] = result_positions(i);
  }
  joint_traj_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(5.0);
  auto joint_traj_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  joint_traj_goal_options.goal_response_callback = [this](const auto &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "IK joint configuration was rejected by server");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "IK joint configuration accepted by server, waiting for result");
    }
  };
  joint_traj_goal_options.result_callback = [this](const auto &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || result.result->error_code != 0) {
      RCLCPP_ERROR(this->get_logger(), "IK joint configuration failed: '%s'", result.result->error_string.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "IK joint configuration successfully executed: '%s'", result.result->error_string.c_str());
  };
  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);
}

void NeuraTaskNode::main_loop_callback()
{
  geometry_msgs::msg::TransformStamped base_link_transform;
  std::string to_frame = "base_link";
  std::string from_frame = "map";
  try {
    base_link_transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", to_frame.c_str(), from_frame.c_str(), ex.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: translation: x: %f, y: %f, z: %f, rotation: x: %f, y: %f, z: %f, w: %f",
    from_frame.c_str(), to_frame.c_str(),
    base_link_transform.transform.translation.x, base_link_transform.transform.translation.y, base_link_transform.transform.translation.z,
    base_link_transform.transform.rotation.x, base_link_transform.transform.rotation.y, base_link_transform.transform.rotation.z, base_link_transform.transform.rotation.w);

  RCLCPP_INFO(this->get_logger(), "Current robot position: x: %f, y: %f, theta: %f",
    base_link_transform.transform.translation.x, base_link_transform.transform.translation.y, 2 * asin(base_link_transform.transform.rotation.z));

  /*constexpr double MAX_ANG = M_PI;
  static size_t cur_step = 0;
  constexpr size_t MAX_STEP = 10;
  std::vector<double> joint_states(6);
  double cur_frac = static_cast<double>(cur_step+1) / static_cast<double>(MAX_STEP);
  joint_states[0] = MAX_ANG * sin(cur_frac * 2 * M_PI);
  move_to_joint_state(joint_states, 1);*/

  //main_loop_timer_->cancel();
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

void NeuraTaskNode::compute_and_move_in_circle()
{
  // move robot to start position
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  geometry_msgs::msg::PoseStamped center;
  center.pose.position.x = get_parameter("circle_center_x").as_double();
  center.pose.position.y = get_parameter("circle_center_y").as_double();
  center.pose.position.z = 0.0;
  center.pose.orientation.x = 0.0;
  center.pose.orientation.y = 0.0;
  center.pose.orientation.z = 0.0;
  center.pose.orientation.w = 1.0;
  waypoints = compute_circle_waypoint(center, get_parameter("circle_radius").as_double(), static_cast<size_t>(get_parameter("circle_num_points").as_int()));

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
}

void NeuraTaskNode::start_task1()
{
  compute_and_move_in_circle();

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

    auto joint_points = compute_sine_joints(get_parameter("sine_joint_mid_values").as_double_array(),
                                            get_parameter("sine_joint_range").as_double_array(),
                                            get_parameter("sine_joint_traj_duration").as_double(),
                                            static_cast<size_t>(get_parameter("sine_joint_traj_steps").as_int()));


    repeat_send_joint_trajectory(joint_points);
  };

  follow_joint_traj_client_->async_send_goal(joint_traj_msg, joint_traj_goal_options);

}