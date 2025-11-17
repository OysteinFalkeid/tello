import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Pose
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose, BoardPose

import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge



class Map_publisher(Node):
    def __init__(self):
        super().__init__("Map_publisher", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.markers_id_list = self.get_parameter("markers").value
        
        self.map_msg = ArucoDetection()
        self.map_msg.header.frame_id = self.get_parameter("header.frame_id").value 
        self.map_msg.header.stamp = self.get_clock().now().to_msg()

        marker_list = []
        for id in self.markers_id_list:
            marker = MarkerPose()
            marker.marker_id = id
            pose = Pose()
            pose.position.x = self.get_parameter_or(f"marker_{id}.pose.position.x", 0.0).value
            pose.position.y = self.get_parameter_or(f"marker_{id}.pose.position.y", 0.0).value
            pose.position.z = self.get_parameter_or(f"marker_{id}.pose.position.z", 0.0).value
            pose.orientation.x = self.get_parameter_or(f"marker_{id}.pose.orientation.x", 0.0).value
            pose.orientation.y = self.get_parameter_or(f"marker_{id}.pose.orientation.y", 0.0).value
            pose.orientation.z = self.get_parameter_or(f"marker_{id}.pose.orientation.z", 0.0).value
            pose.orientation.w = self.get_parameter_or(f"marker_{id}.pose.orientation.w", 0.0).value
            marker.pose = pose
            marker_list.append(marker)

        self.map_msg.markers = marker_list

        self.publisher_map = self.create_publisher(
            msg_type=ArucoDetection,
            topic="aruco_detections_map",
            qos_profile=QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup()
        )


        # publish map
        self.create_timer(
             timer_period_sec=1,
             callback=self.publish_map,
             callback_group=MutuallyExclusiveCallbackGroup()
        )

    def MarkerPose(self, id: int, pose: Pose)->MarkerPose:
        marker = MarkerPose()
        marker.marker_id = id
        marker.pose = pose
        return marker
    
    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_map.publish(self.map_msg)


def main():
    rclpy.init() 
    map_publisher = Map_publisher()

    # Use a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=8)
    if executor.add_node(map_publisher):
        
        try:
            executor.spin()
        finally:
            map_publisher.destroy_node()
            rclpy.shutdown()
    else:
            map_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()