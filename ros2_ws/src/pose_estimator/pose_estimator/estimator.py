import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Pose, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
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

        # publish pose
        # self.create_timer(
        #      timer_period_sec=0.1,
        #      callback=self.publish_pose,
        #      callback_group=MutuallyExclusiveCallbackGroup()
        # )


    def pose_to_matrix(self, pose: Pose):
        """Convert geometry_msgs/Pose → 4x4 transform matrix."""
        trans = tf_transformations.translation_matrix(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        rot = tf_transformations.quaternion_matrix(
            [pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w]
        )
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

    def publish_pose(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_pose.publish(self.pose)

    def get_detections(self, msg: ArucoDetection):
        markers: list[MarkerPose] = msg.markers

        self.markers_odom_detected: dict[int, np.ndarray] = {}
        if len(markers):
            try:
                for marker in markers:

                    # pose = PoseStamped()
                    # pose.header = msg.header
                    # pose.pose = marker.pose
                    
                    # pose_in_odom= self.tf_buffer.transform(
                    #     pose,
                    #     "odom",
                    #     timeout=rclpy.duration.Duration(seconds=0.06)
                    # )

                    matrix_in_odom = self.pose_to_matrix(marker.pose)
                    self.markers_odom_detected[marker.marker_id] = matrix_in_odom
                
                pose_estimation_list = []

                for key in self.markers_odom_detected.keys():
                    if key in self.markers_map.keys():
                        pose_estimaton = self.markers_map[key] @ tf_transformations.inverse_matrix(self.markers_odom_detected[key])
                        pose_estimation_list.append(pose_estimaton)

                message = PoseWithCovarianceStamped()
                message.header.frame_id = "odom"
                message.header.stamp = msg.header.stamp
                message.pose.pose = self.matrix_to_pose(pose_estimation_list[0])
                message.pose.covariance = (np.array([
                    [0.25,     0,        0,        0,         0,         0      ],
                    [0,        0.25,     0,        0,         0,         0      ],
                    [0,        0,        1.0,      0,         0,         0      ],
                    [0,        0,        0,        0.0304,    0,         0      ],
                    [0,        0,        0,        0,         0.0304,    0      ],
                    [0,        0,        0,        0,         0,         0.1212 ]
                    ]) * 1).flatten().tolist()
                
                self.publisher_pose.publish(message)


            except Exception as e:
                    self.get_logger().warning(f"{e}")

    def get_map(self, msg: ArucoDetection):
        markers: list[MarkerPose] = msg.markers

        self.markers_map: dict[int, np.ndarray] = {}

        for marker in markers:
            self.markers_map[marker.marker_id] =  self.pose_to_matrix(marker.pose)
        

        # marker_base_link_list: list[MarkerPose] = []
        # for marker in markers:
        #     self.map_pose = PoseStamped()
        #     self.map_pose.header = msg.header
        #     self.map_pose.pose = marker.pose

        #     try:
        #         # Transform map → base_link
        #         pose_in_baselink = self.tf_buffer.transform(
        #             self.map_pose,
        #             "base_link",
        #             timeout=rclpy.duration.Duration(seconds=1.0)
        #         )
        #         # self.get_logger().info(f"{pose_in_baselink}")
        #         marker_base_link = MarkerPose()
        #         marker_base_link.marker_id = marker.marker_id
        #         marker_base_link.pose = pose_in_baselink.pose
        #         marker_base_link_list.append(marker_base_link)
        #     except Exception as e:
        #         self.get_logger().warning(f"{e}")
        
        # self.map.header = msg.header
        # self.map.markers = marker_base_link_list

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