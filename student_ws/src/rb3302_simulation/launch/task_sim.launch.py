"""Automatic task run using the same Gazebo and Nav2 launches as manual use."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    sim = get_package_share_directory('rb3302_simulation')
    nav = get_package_share_directory('rb3302_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '72'),
        SetEnvironmentVariable('GZ_PARTITION', 'rb3302_sim'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim, 'launch', 'simulation.launch.py'])),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        ExecuteProcess(
            cmd=['python3', PathJoinSubstitution([sim, 'launch', 'task_nav_gate.py']),
                 '--rviz', LaunchConfiguration('rviz')], output='screen'),
        ExecuteProcess(
            cmd=['python3', PathJoinSubstitution([sim, 'launch', 'task_readiness.py']),
                 '--task-file', PathJoinSubstitution([nav, 'config', 'task.yaml']),
                 '--landmarks-file', PathJoinSubstitution([sim, 'config', 'landmarks_sim.yaml'])],
            output='screen'),
    ])
