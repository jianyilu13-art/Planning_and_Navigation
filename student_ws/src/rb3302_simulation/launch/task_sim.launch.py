"""Complete Gazebo/RPP/four-target test using simulation-only calibration."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('rb3302_simulation')
    nav = get_package_share_directory('rb3302_navigation')
    rpp = get_package_share_directory('planning_rpp')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim, 'launch', 'simulation.launch.py'])),
            launch_arguments={'gui': 'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim, 'launch', 'sim_navigation.launch.py'])),
            launch_arguments={
                'rviz': 'false',
                'params_file': PathJoinSubstitution([rpp, 'config', 'nav2_params.yaml']),
            }.items()),
        # The action server exists before Nav2 finishes activating its
        # costmaps; defer the one-shot task so its first goal is not rejected.
        TimerAction(period=12.0, actions=[
            Node(package='rb3302_navigation', executable='task_manager', output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'task_file': PathJoinSubstitution([nav, 'config', 'task.yaml']),
                     'landmarks_file': PathJoinSubstitution([sim, 'config', 'landmarks_sim.yaml']),
                     'goal_xy_tolerance': 0.30,
                 }]),
        ]),
    ])
