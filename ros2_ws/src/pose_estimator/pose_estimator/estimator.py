import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Pose, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose, BoardPose
import tf2_ros
import tf2_geometry_msgs
import tf_transformations

import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import math



class Estimator(Node):
    def __init__(self):
        super().__init__("Estimator")

        self.pose = PoseWithCovarianceStamped()
        self.pose.header.frame_id = "odom"
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.covariance = np.array([
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        ]).flatten().tolist()
        
        self.map = ArucoDetection()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.markers_map: dict[int, np.ndarray] = {}
        self.markers_odom_detected: dict[int, np.ndarray] = {}


        matrix = self.pose_to_matrix(Pose())
        self.odometry_matrix = matrix

        self.publisher_pose = self.create_publisher(
             msg_type=PoseWithCovarianceStamped,
             topic="pose0_landmarks",
             qos_profile=QoSProfile(depth=10),
             callback_group=MutuallyExclusiveCallbackGroup()
        )

        #subscribe detections
        self.create_subscription(
            msg_type=ArucoDetection,
            topic="aruco_detections",
            qos_profile=QoSProfile(depth=10),
            callback=self.get_detections,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #subscribe map
        self.create_subscription(
            msg_type=ArucoDetection,
            topic="aruco_detections_map",
            qos_profile=QoSProfile(depth=10),
            callback=self.get_map,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #subscribe odometry filteres
        self.create_subscription(
            msg_type=Odometry,
            topic="control/odometry/filtered",
            qos_profile=QoSProfile(depth=10),
            callback=self.subscribe_to_odometry_filtered,
            callback_group=MutuallyExclusiveCallbackGroup()
        )


    def pose_to_matrix(self, pose: Pose):
        """Convert geometry_msgs/Pose → 4x4 transform matrix."""
        trans = tf_transformations.translation_matrix(
            [pose.position.x, 
             pose.position.y, 
             pose.position.z]
        )

        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        # Convert quaternion → Euler angles
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)

        # Zero roll, pitch
        roll = 0.0
        pitch = 0.0

        # Recompute quaternion from yaw-only rotation
        q_z_only = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        rot = tf_transformations.quaternion_matrix(q_z_only)

        return trans @ rot

    def matrix_to_pose(self, T: np.ndarray):
        """Convert 4x4 transform matrix → geometry_msgs/Pose."""
        pose = Pose()  # temporary to get correct type

        # Extract translation
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]

        # Extract quaternion
        q = tf_transformations.quaternion_from_matrix(T)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    # def publish_pose(self):
    #     self.pose.header.stamp = self.get_clock().now().to_msg()
    #     self.publisher_pose.publish(self.pose)

    def subscribe_to_odometry_filtered(self, msg: Odometry):
        matrix = self.pose_to_matrix(msg.pose.pose)
        self.odometry_matrix = matrix

    def get_detections(self, msg: ArucoDetection):
        markers: list[MarkerPose] = msg.markers

        self.markers_odom_detected: dict[int, np.ndarray] = {}
        if len(markers):
            try:
                for marker in markers:

                    matrix_in_odom = self.pose_to_matrix(marker.pose)
                    self.markers_odom_detected[marker.marker_id] = matrix_in_odom
                
                pose_estimation_list = []
                error_list = []

                for key in self.markers_odom_detected.keys():
                    if key in self.markers_map.keys():
                        T_odom_marker = self.markers_map[key]
                        T_drone_marker = self.markers_odom_detected[key]

                        # Marker-based drone pose in odom
                        T_odom_drone_aruco = T_odom_marker @ tf_transformations.inverse_matrix(T_drone_marker)
                        pose_estimation_list.append(T_odom_drone_aruco)

                        # Difference between odom EKF pose and ArUco-based pose
                        delta = tf_transformations.inverse_matrix(self.odometry_matrix) @ T_odom_drone_aruco
                        error_list.append(delta)


                for i, estimation in enumerate(pose_estimation_list):

                    delta = error_list[i]

                    pos_err = math.sqrt(
                        delta[0, 3]**2 +
                        delta[1, 3]**2 +
                        delta[2, 3]**2
                    )

                    cov = (np.array([
                        [10.0, 0,    0,    0,    0,    0],
                        [0,    10.0, 0,    0,    0,    0],
                        [0,    0,    10.0, 0,    0,    0],
                        [0,    0,    0,    10.0, 0,    0],
                        [0,    0,    0,    0,    10.0, 0],
                        [0,    0,    0,    0,    0,    10.0],
                    ]) * max(1.0, pos_err * 4.0)).astype(float).flatten().tolist()

                    message = PoseWithCovarianceStamped()
                    message.header.frame_id = "odom"
                    message.header.stamp = msg.header.stamp
                    message.pose.pose = self.matrix_to_pose(estimation)
                    message.pose.covariance = cov

                    self.publisher_pose.publish(message)



            except Exception as e:
                    self.get_logger().warning(f"{e}")

    def get_map(self, msg: ArucoDetection):

        markers: list[MarkerPose] = msg.markers

        self.markers_map: dict[int, np.ndarray] = {}

        for marker in markers:
            self.markers_map[marker.marker_id] =  self.pose_to_matrix(marker.pose)

def main():
    rclpy.init() 
    estimator = Estimator()

    # Use a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=8)
    if executor.add_node(estimator):
        
        try:
            executor.spin()
        finally:
            estimator.destroy_node()
            rclpy.shutdown()
    else:
            estimator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()