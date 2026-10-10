from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('planning_astar')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    autostart = LaunchConfiguration('autostart')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'amcl.ros__parameters.global_frame_id': map_frame,
                'bt_navigator.ros__parameters.global_frame': map_frame,
                'local_costmap.local_costmap.ros__parameters.global_frame': odom_frame,
                'global_costmap.global_costmap.ros__parameters.global_frame': map_frame,
                'behavior_server.ros__parameters.global_frame': map_frame,
                'behavior_server.ros__parameters.local_frame': odom_frame,
                'robot_base_frame': base_frame,
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'topic': scan_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [EnvironmentVariable('HOME'), 'rb3302_final_proj_code', 'student_ws', 'maps', 'rb3302_map.yaml'])),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [share, 'rviz', 'navigation.rviz'])),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('raw_odom_topic', default_value='/odom/raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu_data_ros'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frequency': 50.0,
                'sensor_timeout': 0.30,
                'two_d_mode': True,
                'transform_timeout': 0.10,
                'print_diagnostics': True,
                'publish_tf': True,
                'publish_acceleration': False,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_link_frame': base_frame,
                'world_frame': odom_frame,
                'odom0': raw_odom_topic,
                'odom0_config': [
                    False, False, False,
                    False, False, False,
                    True, False, False,
                    False, False, False,
                    False, False, False,
                ],
                'odom0_queue_size': 10,
                'odom0_differential': False,
                'odom0_relative': False,
                'odom0_pose_rejection_threshold': 5.0,
                'odom0_twist_rejection_threshold': 5.0,
                'imu0': imu_topic,
                'imu0_config': [
                    False, False, False,
                    False, False, True,
                    False, False, False,
                    False, False, True,
                    False, False, False,
                ],
                'imu0_queue_size': 50,
                'imu0_differential': False,
                'imu0_relative': True,
                'imu0_remove_gravitational_acceleration': False,
                'imu0_pose_rejection_threshold': 5.0,
                'imu0_twist_rejection_threshold': 5.0,
            }],
            remappings=[('odometry/filtered', odom_topic)],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                configured_params,
                {
                    'set_initial_pose': set_initial_pose,
                    'initial_pose': {
                        'x': initial_pose_x,
                        'y': initial_pose_y,
                        'z': 0.0,
                        'yaw': initial_pose_yaw,
                    },
                },
            ],
            remappings=[('scan', scan_topic)],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav_raw'), ('odom', odom_topic)],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', cmd_vel_topic),
            ],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', cmd_vel_topic)],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=[('odom', odom_topic)],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
