import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path)
    )


def generate_launch_description():
    return LaunchDescription([
        include_launch('yesense_std_ros2', 'yesense_node.launch.py'),
        include_launch('rplidar_ros', 'rplidar_launch.py'),
        include_launch('robot_model', 'description.launch.py'),
        include_launch('waverover_base', 'driver.launch.py'),
    ])
