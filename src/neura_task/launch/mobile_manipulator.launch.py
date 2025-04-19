from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ur_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ur_simulation_gz'),
                        'launch',
                        'ur_sim_control.launch.py'
                    ])
                ]),
                launch_arguments={
                    'description_file': PathJoinSubstitution([FindPackageShare("neura_task"), "urdf", "mobile_manipulator.urdf.xacro"])
                }.items()
        )
    
    task_node = Node(
            package='neura_task',
            executable='neura_task_node',
            name='neura_task_node')

    return LaunchDescription([
        ur_launch,
        task_node,
  ])