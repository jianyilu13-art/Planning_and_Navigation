"""Gazebo-only Wave Rover simulation; no physical drivers are launched."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
def generate_launch_description():
 s=get_package_share_directory('rb3302_simulation'); gz=get_package_share_directory('ros_gz_sim')
 return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'), SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([s, 'models'])), IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([gz,'launch','gz_sim.launch.py'])),launch_arguments={'gz_args':['-r ',PathJoinSubstitution([s,'worlds','rb3302.world.sdf'])]}.items(), condition=IfCondition(LaunchConfiguration('gui'))), IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([gz,'launch','gz_sim.launch.py'])),launch_arguments={'gz_args':['-r -s ',PathJoinSubstitution([s,'worlds','rb3302.world.sdf'])]}.items(), condition=UnlessCondition(LaunchConfiguration('gui'))), Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':PathJoinSubstitution([s,'config','bridge.yaml'])}],output='screen'), Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':open(__file__.replace('launch/simulation.launch.py','models/simulated_waverover/model.urdf')).read()}],output='screen'), Node(package='tf2_ros', executable='static_transform_publisher', arguments=['-0.052','0','0.125','0','0','0','base_link','waverover/base_link/lidar'], parameters=[{'use_sim_time':True}])])
