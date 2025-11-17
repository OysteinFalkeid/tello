import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, PoseStamped, Pose, Vector3Stamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs 

import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
import time



class PID(Node):
    def __init__(self):
        super().__init__("PID")
        
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "odom"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.wait_for_transform("odom", "base_link")
        self.transform = self.tf_buffer.lookup_transform(
            target_frame='base_link',
            source_frame='odom',
            time=rclpy.time.Time()
        )


        self.create_subscription(
            msg_type=PoseStamped,
            topic="goal_pose",
            callback=self.subscribe_goal,
            callback_group=MutuallyExclusiveCallbackGroup(),
            qos_profile=QoSProfile(depth=10),
        )

        self.create_subscription(
            msg_type=Odometry,
            topic="odometry/filtered",
            callback=self.subscribe_base_link,
            callback_group=MutuallyExclusiveCallbackGroup(),
            qos_profile=QoSProfile(depth=10),
        )

        self.publisher_cmd_vel = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.create_timer(
            timer_period_sec=0.05,
            callback=self.get_transform,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    
    def wait_for_transform(self, source_frame, target_frame):
        self.get_logger().info(f"Waiting for transform {source_frame} → {target_frame}...")

        while rclpy.ok():
            try:
                # Jazzy-appropriate transform lookup WITH timeout
                _ = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),                        # latest available
                    timeout=rclpy.time.Duration(seconds=0.5)
                )
                self.get_logger().info("Transform available!")
                return None
            except Exception as e:
                # pass
                time.sleep(0.5)
                self.get_logger().info("Transform not ready yet...")
                self.get_logger().info(f"{e}")


    def get_transform(self):
        try:
            self.transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame='odom',
                time=rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=0.5)
            )
        except:
            pass

    def subscribe_goal(self, msg:PoseStamped):
        self.goal_pose = msg

        goal_pose: PoseStamped = tf2_geometry_msgs.do_transform_pose_stamped(self.goal_pose, self.transform).pose

        x = -goal_pose.pose.position.x
        y = -goal_pose.pose.position.y
        z = -goal_pose.pose.position.z


        psi = -self.pose_to_psi(goal_pose.pose)

        twist_array = np.array([x, y, z, psi]).astype(float)

        twist_array = twist_array / 1.0

        for i, value in enumerate(twist_array):
            if value > 1.0:
                twist_array[i] = 1.0
            elif value < -1.0:
                twist_array[i] = 1.0
        
        message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = msg.header.stamp

        message.twist.linear.x = twist_array[0]
        message.twist.linear.y = twist_array[1]
        message.twist.linear.z = twist_array[2]

        message.twist.angular.z = twist_array[3]

        self.publisher_cmd_vel.publish(message)

    def pose_to_psi(self, pose: Pose):
        return math.atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1 - 2*(pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z))
    
    def subscribe_base_link(self, msg: Odometry):
        odometry_pose = msg.pose.pose
        












def main():
    rclpy.init() 
    pid = PID()

    # Use a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=8)
    if executor.add_node(pid):
        
        try:
            executor.spin()
        finally:
            pid.destroy_node()
            rclpy.shutdown()
    else:
            pid.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()