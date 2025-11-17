import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    tello_node = Node(
        package="tello",
        executable="tello",
        name="tello_api",
        namespace="tello",
        output="screen",
    )

    camera_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_transform",
        output="screen",
        arguments=["0.03", "0", "0", "-1.57079632679", "0", "-1.57079632679", "base_link","camera"]
    )


    return LaunchDescription([
        tello_node,
        camera_transform_node,
    ])