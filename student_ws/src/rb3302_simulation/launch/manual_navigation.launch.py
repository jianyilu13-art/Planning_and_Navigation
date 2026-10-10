"""Manual Nav2 + RViz; start simulation.launch.py separately first."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('rb3302_simulation')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sim_navigation.launch.py'])))])
