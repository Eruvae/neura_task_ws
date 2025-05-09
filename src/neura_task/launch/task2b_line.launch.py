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
                {"task": "task2b"},
                {"task2b_base_move_type": "line"},
                {"task2b_endeffector_pose": [0.207, 0.743, 1.290, -0.478, 0.529, -0.501, 0.491]},
                {"line_start_x": 0.0},
                {"line_start_y": 0.5},
                {"line_start_theta": 0.0},
                {"line_end_x": 0.0},
                {"line_end_y": 1.0},
                {"line_end_theta": 0.0}
            ])

    return LaunchDescription([
        task_node
  ])