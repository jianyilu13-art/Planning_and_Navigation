"""One coherent Nav2 stack plus the four-waypoint task manager."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rb3302_navigation')
    planning = get_package_share_directory('planning_rpp')
    arg = lambda name, default: DeclareLaunchArgument(name, default_value=default)
    return LaunchDescription([
        arg('start_hardware', 'false'), arg('use_sim_time', 'false'), arg('rviz', 'true'),
        # The supplied map is deliberately outside this package and is never copied or altered.
        arg('map', '/ssd/workspaces/rb3302_final_proj/student_ws/maps/rb3302_map.yaml'),
        arg('params_file', PathJoinSubstitution([share, 'config', 'nav2_real.yaml'])),
        arg('landmarks_file', PathJoinSubstitution([share, 'config', 'landmarks.yaml'])),
        arg('task_file', PathJoinSubstitution([share, 'config', 'task.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('robot_init'), 'launch', 'robot.launch.py'])), condition=IfCondition(LaunchConfiguration('start_hardware'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([planning, 'launch', 'planning.launch.py'])), launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'), 'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('params_file'), 'rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='rb3302_navigation', executable='task_manager', output='screen', parameters=[{
            'task_file': LaunchConfiguration('task_file'), 'landmarks_file': LaunchConfiguration('landmarks_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
