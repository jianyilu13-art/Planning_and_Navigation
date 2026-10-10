from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_setup(context):
    mapping_share = get_package_share_directory('mapping_cartographer')
    value = lambda name: LaunchConfiguration(name).perform(context)
    use_sim_time = value('use_sim_time').lower() == 'true'
    normalize_scan_time = value('normalize_scan_time').lower() == 'true'
    rviz = value('rviz').lower() == 'true'
    scan_topic = value('scan_topic')
    normalized_scan_topic = value('normalized_scan_topic')
    imu_topic = value('imu_topic')
    configuration_basename = value('configuration_basename')

    cartographer_scan_topic = (
        normalized_scan_topic if normalize_scan_time else scan_topic)
    print(
        '[mapping_cartographer] config=%s, odometry=%s, scan=%s' % (
            configuration_basename, False, cartographer_scan_topic))

    actions = []

    if normalize_scan_time:
        actions.append(Node(
            package='mapping_cartographer',
            executable='scan_time_normalizer',
            name='scan_time_normalizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': scan_topic,
                'output_topic': normalized_scan_topic,
            }],
        ))

    actions.extend([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', value('configuration_directory'),
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('scan', cartographer_scan_topic),
                ('imu', imu_topic),
                ('odom', value('odom_topic')),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', value('resolution'),
                '-publish_period_sec', '1.0',
            ],
        ),
    ])
    if rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='mapping_rviz',
            output='screen', arguments=['-d', value('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('mapping_cartographer')
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('normalized_scan_topic',
                              default_value='/scan_cartographer'),
        DeclareLaunchArgument('imu_topic', default_value='/imu_data_ros'),
        DeclareLaunchArgument(
            'normalize_scan_time', default_value='true',
            description='Cap overlapping 2D RP Lidar per-beam timestamps'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=PathJoinSubstitution([share, 'config'])),
        DeclareLaunchArgument(
            'configuration_basename', default_value='cartographer.lua'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([share, 'rviz', 'mapping.rviz'])),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
