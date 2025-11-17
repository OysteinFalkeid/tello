import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    aruco_map_params = PathJoinSubstitution([
        get_package_share_directory("aruco_opencv_map"),
        "params",
        "example_map.yaml",
    ])

    aruco_opencv_map_node = Node(
        package="aruco_opencv_map",
        executable="aruco_opencv_map_publisher",
        name="aruco_opencv_map",
        output="screen",
        parameters=[aruco_map_params]
    )


    return LaunchDescription([
        aruco_opencv_map_node,
    ])