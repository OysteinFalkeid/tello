import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    teleop_config = PathJoinSubstitution([
        get_package_share_directory('drone'),
        'params',
        'teleop_joy_config_8bitdo.yaml'
        ])
    
    joy_config = PathJoinSubstitution([
        get_package_share_directory('drone'),
        'params',
        'joy_teleop_config_8bitdo.yaml'
        ])

    localization_config = PathJoinSubstitution([
        get_package_share_directory("drone"), 
        'params', 
        'ekf_config.yaml'
        ])
    
    aruco_tracker_params = PathJoinSubstitution([
        get_package_share_directory("drone"), 
        'params', 
        'aruco_tracker_config.yaml'
        ])
    
    # aruco_map_params = PathJoinSubstitution([
    #     get_package_share_directory("drone"),
    #     "params",
    #     "example_map.yaml",
    # ])

    aruco_map_params = PathJoinSubstitution([
        get_package_share_directory("drone"),
        "params",
        "competition_map_config.yaml",
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', PathJoinSubstitution((get_package_share_directory("drone"), "params", "rviz2.rviz"))]
    )

    tello_api_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution((
                get_package_share_directory("tello"),
                "launch",
                "tello.launch.py"
            ))
        )
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        namespace="/tello/control",
        output='screen',
    )
    
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace="tello/control",
        output='screen',
        parameters=[joy_config]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        namespace="tello/control",
        output='screen',
        remappings=[
            ("cmd_vel", "joy_cmd_vel")
        ],
        parameters=[teleop_config]

    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace="tello/control",
        output='screen',
        parameters=[localization_config],
        )

    map_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_transform",
        output="screen",
        arguments=["0","0","0","0","0","0","map","odom"]
        )
    
    odom_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_transform",
        output="screen",
        arguments=["0","0","0","0","0","0","odom","base_link"]
        )

    aruco_tracker_node = Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            namespace="tello",
            output='screen',            
            parameters=[aruco_tracker_params],
        )

    aruco_opencv_map_publisher_node = Node(
        package="aruco_opencv_map",
        executable="aruco_opencv_map_publisher",
        name="aruco_opencv_map",
        namespace="tello",
        output="screen",
        parameters=[aruco_map_params]
    )

    pose_estimator_node = Node(
        package="pose_estimator",
        executable="estimator",
        name="pose_estimator",
        namespace="tello",
        output="screen",
        remappings=[
            ("pose0_landmarks", "control/pose0_landmarks")
        ]
    )

    pid_controller_node = Node(
        package="pid_controller",
        executable="pid",
        name="pid_controller",
        namespace="tello/control",
        output="screen",
    )
    
    pose_publusher_node = Node(
        package="pose_publisher",
        executable="pose_publisher",
        name="pose_publisher",
        namespace="tello/control",
        output="screen",
    )


    return LaunchDescription([
        tello_api_node,
        rviz2_node,
        joy_node,
        joy_teleop_node,
        teleop_twist_joy_node,
        robot_localization_node,
        aruco_tracker_node,
        map_transform_node,
        odom_transform_node,
        aruco_opencv_map_publisher_node,
        pose_estimator_node,
        pid_controller_node,
        pose_publusher_node,
    ])
