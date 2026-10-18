"""Nav2 adapter for Gazebo, retaining the supplied Navfn/RPP configuration.

Gazebo is the sole publisher of ``odom -> base_link``; AMCL publishes
``map -> odom``. The physical-sensor EKF is not started in simulation.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    sim = get_package_share_directory('rb3302_simulation')
    nav = get_package_share_directory('rb3302_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Do not run this YAML through RewrittenYaml: its type conversion has
    # previously converted costmap dimensions into integers on this system.
    params = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    common = [params, {'use_sim_time': use_sim_time}]
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([sim, 'config', 'nav2_sim.yaml'])),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([sim, 'maps', 'rb3302_sim_map.yaml'])),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', parameters=[params, {'use_sim_time': use_sim_time, 'yaml_filename': map_file}]),
        # The simulated rover is spawned at landmark A's centre. Supplying that
        # pose lets AMCL establish map -> odom for standalone/manual Nav2
        # testing, where task_manager deliberately is not launched.
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time,
                                  'set_initial_pose': True,
                                  'initial_pose': {'x': 0.3, 'y': 2.1, 'z': 0.0, 'yaw': 0.0}}],
             remappings=[('scan', '/scan')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': ['map_server', 'amcl']}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav_raw'), ('odom', '/odom')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav_raw'), ('cmd_vel_smoothed', '/cmd_vel')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=common, remappings=[('cmd_vel', '/cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=common, remappings=[('odom', '/odom')]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': ['planner_server', 'controller_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', PathJoinSubstitution([sim, 'rviz', 'simulation.rviz'])], parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
