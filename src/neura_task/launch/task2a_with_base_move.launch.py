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
                {"task": "task2a"},
                {"task2a_start_pose": [-1.662, -0.192, 1.162, 0.746, -0.007, -0.011, 0.666]},
                {"task2a_end_pose": [-2.181, 0.098, 1.162, 0.690, -0.282, -0.257, 0.615]},
                {"task2a_linear_velocity": 0.1},
                {"task2a_linear_acceleration": 0.1},
                {"task2a_step_size": 0.01}
            ])

    return LaunchDescription([
        task_node
  ])