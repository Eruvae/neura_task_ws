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
                {"task2b_base_move_type": "circle"},
                {"task2b_endeffector_pose": [-1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0]}
            ])

    return LaunchDescription([
        task_node
  ])