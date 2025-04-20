from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    task_node = Node(
            package='neura_task',
            executable='neura_task_node',
            name='neura_task_node',
            parameters=[
                {"use_sim_time": True},
                {"task": "task1"},
                {"circle_center_x": -1.0},
                {"circle_center_y": 0.5},
                {"circle_radius": 0.75},
                {"circle_num_points": 16},
                {"sine_joint_mid_values": [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]},
                {"sine_joint_range": [0.785, 0.785, 0.785, 0.785, 0.785, 0.785]},
                {"sine_joint_traj_duration": 10.0},
                {"sine_joint_traj_steps": 100}
            ])

    return LaunchDescription([
        task_node
  ])