#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
import tf2_ros
import tf_transformations

import numpy as np
import math


class Estimator(Node):
    def __init__(self):
        super().__init__("Estimator")

        # Initial pose message (not really used actively here, but kept)
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

        # EKF odometry pose as 4x4 matrix
        self.odometry_matrix = self.pose_to_matrix(Pose())

        # Pose filter state for ArUco-based drone pose (4x4 matrix)
        self.last_aruco_pose_T = None

        # Publisher for landmark-corrected pose
        self.publisher_pose = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic="pose0_landmarks",
            qos_profile=QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Subscribe to detections
        self.create_subscription(
            msg_type=ArucoDetection,
            topic="aruco_detections",
            qos_profile=QoSProfile(depth=10),
            callback=self.get_detections,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Subscribe to static map of marker poses (in odom)
        self.create_subscription(
            msg_type=ArucoDetection,
            topic="aruco_detections_map",
            qos_profile=QoSProfile(depth=10),
            callback=self.get_map,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Subscribe to filtered odometry (from EKF)
        self.create_subscription(
            msg_type=Odometry,
            topic="control/odometry/filtered",
            qos_profile=QoSProfile(depth=10),
            callback=self.subscribe_to_odometry_filtered,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    # ------------------------------------------------------------------
    #  Helpers: Pose ↔ matrix, extracting and building xyz + yaw
    # ------------------------------------------------------------------

    def pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """Convert geometry_msgs/Pose → 4x4 transform matrix, zeroing roll/pitch."""
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

        # quaternion → Euler
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)

        # ignore roll/pitch, keep yaw only
        roll = 0.0
        pitch = 0.0

        q_z_only = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        rot = tf_transformations.quaternion_matrix(q_z_only)

        return trans @ rot

    def matrix_to_pose(self, T: np.ndarray) -> Pose:
        """Convert 4x4 transform matrix → geometry_msgs/Pose."""
        pose = Pose()
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]

        q = tf_transformations.quaternion_from_matrix(T)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def extract_xyz_yaw(self, T: np.ndarray):
        """Extract x, y, z, yaw from a 4x4 transform matrix."""
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        _, _, yaw = tf_transformations.euler_from_matrix(T)
        return x, y, z, yaw

    def build_T_from_xyz_yaw(self, x: float, y: float, z: float, yaw: float) -> np.ndarray:
        """Build a 4x4 transform from x, y, z, yaw (roll = pitch = 0)."""
        T = tf_transformations.translation_matrix([x, y, z])
        Rz = tf_transformations.euler_matrix(0.0, 0.0, yaw)
        T[0:3, 0:3] = Rz[0:3, 0:3]
        return T

    # ------------------------------------------------------------------
    #  Full pose filter for ArUco (x, y, z, yaw)
    # ------------------------------------------------------------------

    def filter_aruco_pose(self, T_raw: np.ndarray) -> np.ndarray:
        """
        Filter the ArUco-based pose in odom (x, y, z, yaw).
        - Low-pass on position and yaw.
        - Step limit on position and yaw to avoid crazy jumps.
        """
        x, y, z, yaw = self.extract_xyz_yaw(T_raw)

        if self.last_aruco_pose_T is None:
            # First measurement: accept as is.
            T_filt = self.build_T_from_xyz_yaw(x, y, z, yaw)
            self.last_aruco_pose_T = T_filt
            return T_filt

        x_prev, y_prev, z_prev, yaw_prev = self.extract_xyz_yaw(self.last_aruco_pose_T)

        # deltas
        dx = x - x_prev
        dy = y - y_prev
        dz = z - z_prev

        # wrap-aware yaw delta
        dyaw = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))

        # ---- Step limits ----
        max_step_pos = 0.10   # [m] per update axis-wise (tune)
        max_step_z   = 0.10   # [m] per update in z (tune)
        max_step_yaw = math.radians(25.0)  # [rad] per update (tune)

        dx = max(-max_step_pos, min(max_step_pos, dx))
        dy = max(-max_step_pos, min(max_step_pos, dy))
        dz = max(-max_step_z,   min(max_step_z,   dz))

        if abs(dyaw) > max_step_yaw:
            dyaw = max_step_yaw * math.copysign(1.0, dyaw)

        # ---- Low-pass filters ----
        alpha_pos = 0.4  # 0<alpha<=1, lower = smoother
        alpha_z   = 0.4
        alpha_yaw = 0.2

        x_f = x_prev + alpha_pos * dx
        y_f = y_prev + alpha_pos * dy
        z_f = z_prev + alpha_z   * dz
        yaw_f = yaw_prev + alpha_yaw * dyaw

        # wrap yaw back to [-pi, pi]
        yaw_f = math.atan2(math.sin(yaw_f), math.cos(yaw_f))

        T_filt = self.build_T_from_xyz_yaw(x_f, y_f, z_f, yaw_f)
        self.last_aruco_pose_T = T_filt
        return T_filt

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------

    def publish_pose(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_pose.publish(self.pose)

    def subscribe_to_odometry_filtered(self, msg: Odometry):
        # Store EKF odom pose as matrix (yaw-only)
        self.odometry_matrix = self.pose_to_matrix(msg.pose.pose)

    def get_detections(self, msg: ArucoDetection):
        markers: list[MarkerPose] = msg.markers

        self.markers_odom_detected = {}
        if not markers:
            return

        try:
            for marker in markers:
                matrix_in_odom = self.pose_to_matrix(marker.pose)
                self.markers_odom_detected[marker.marker_id] = matrix_in_odom

            pose_estimation_list = []
            error_list = []

            for key, T_drone_marker in self.markers_odom_detected.items():
                if key not in self.markers_map:
                    continue

                T_odom_marker = self.markers_map[key]

                # Drone pose from this marker in odom
                T_odom_drone_aruco = T_odom_marker @ tf_transformations.inverse_matrix(T_drone_marker)

                # ---- Filter the full pose (x, y, z, yaw) ----
                T_odom_drone_filt = self.filter_aruco_pose(T_odom_drone_aruco)
                pose_estimation_list.append(T_odom_drone_filt)

                # Difference between EKF odom pose and filtered ArUco pose
                delta = tf_transformations.inverse_matrix(self.odometry_matrix) @ T_odom_drone_filt
                error_list.append(delta)

            for i, estimation in enumerate(pose_estimation_list):
                delta = error_list[i]

                pos_err = math.sqrt(
                    delta[0, 3] ** 2 +
                    delta[1, 3] ** 2 +
                    delta[2, 3] ** 2
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
                message.header.frame_id = "base_link"
                message.header.stamp = msg.header.stamp
                message.pose.pose = self.matrix_to_pose(estimation)
                message.pose.covariance = cov

                self.publisher_pose.publish(message)

        except Exception as e:
            self.get_logger().warning(f"{e}")

    def get_map(self, msg: ArucoDetection):
        markers: list[MarkerPose] = msg.markers
        self.markers_map = {}

        for marker in markers:
            self.markers_map[marker.marker_id] = self.pose_to_matrix(marker.pose)


def main():
    rclpy.init()
    estimator = Estimator()

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
