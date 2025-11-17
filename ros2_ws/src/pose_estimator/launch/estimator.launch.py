import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    estimator_params = PathJoinSubstitution([
        get_package_share_directory("pose_estimator"),
        "params",
        "example_map.yaml",
    ])

    estimator_node = Node(
        package="pose_estimator",
        executable="estimatoe",
        name="pose_estimator",
        output="screen",
        parameters=[estimator_params]
    )


    return LaunchDescription([
            estimator_node,
    ])