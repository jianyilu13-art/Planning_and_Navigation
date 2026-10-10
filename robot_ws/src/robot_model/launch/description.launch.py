"""Publishes the robot_description and starts robot_state_publisher.

robot_state_publisher is responsible for
base_link -> imu_tracking -> gyro_link -> laser
(and any other fixed-joint) transforms defined in the URDF/xacro model.
It does NOT publish odom -> base_link (that comes from the external odometry
EKF) nor map -> odom (that comes from AMCL).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_model')
    default_xacro_path = PathJoinSubstitution(
        [pkg_share, 'urdf', 'diffbot.urdf.xacro']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    base_frame = LaunchConfiguration('base_frame')
    tracking_frame = LaunchConfiguration('tracking_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_yaw = LaunchConfiguration('laser_yaw')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    publish_frequency = LaunchConfiguration('publish_frequency')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_model = DeclareLaunchArgument(
        'model', default_value=default_xacro_path,
        description='Absolute path to the robot xacro/urdf file'
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Name of the robot base frame'
    )
    declare_laser_frame = DeclareLaunchArgument(
        'laser_frame', default_value='laser',
        description='Name of the 2D LiDAR frame'
    )
    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame', default_value='gyro_link',
        description='Name of the H30 IMU frame'
    )
    declare_tracking_frame = DeclareLaunchArgument(
        'tracking_frame', default_value='imu_tracking',
        description='FLU Cartographer tracking frame at the H30 origin'
    )
    declare_laser_yaw = DeclareLaunchArgument(
        'laser_yaw', default_value='3.141592653589793',
        description=(
            'Yaw (radians) of the LiDAR scan frame relative to base_link forward. '
            'This robot defaults to a rear-facing scan-zero direction (pi rad).')
    )
    declare_use_jsp = DeclareLaunchArgument(
        'use_joint_state_publisher', default_value='false',
        description='Launch joint_state_publisher (not required: all joints are fixed)'
    )
    declare_publish_frequency = DeclareLaunchArgument(
        'publish_frequency', default_value='30.0',
        description='robot_state_publisher TF publish frequency (Hz)'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro ', model,
            ' base_frame:=', base_frame,
            ' tracking_frame:=', tracking_frame,
            ' imu_frame:=', imu_frame,
            ' laser_frame:=', laser_frame,
            ' laser_yaw:=', laser_yaw,
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': publish_frequency,
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state_publisher),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_base_frame,
        declare_tracking_frame,
        declare_imu_frame,
        declare_laser_frame,
        declare_laser_yaw,
        declare_use_jsp,
        declare_publish_frequency,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
