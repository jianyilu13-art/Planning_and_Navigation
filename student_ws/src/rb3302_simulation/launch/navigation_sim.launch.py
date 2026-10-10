"""Gazebo, the simulation Nav2 adapter, and the existing task manager."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('rb3302_simulation')
    nav = get_package_share_directory('rb3302_navigation')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim, 'launch', 'simulation.launch.py']))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim, 'launch', 'sim_navigation.launch.py']))),
        Node(package='rb3302_navigation', executable='task_manager', output='screen',
             parameters=[{'use_sim_time': True,
                          'task_file': PathJoinSubstitution([nav, 'config', 'task.yaml']),
                          'landmarks_file': PathJoinSubstitution([nav, 'config', 'landmarks.yaml'])}]),
    ])
