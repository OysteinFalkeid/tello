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

        self.x_test = 0.1
        self.z_test = 0.0
        
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "odom"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.z = 0.89

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.secunds = 0.0
        self.twist_array = np.array([0, 0, 0, 0]).astype(float)
        self.twist_array_1 = np.array([0, 0, 0, 0]).astype(float)
        self.twist_array_filter= [np.array([0, 0, 0, 0]).astype(float), np.array([0, 0, 0, 0]).astype(float), np.array([0, 0, 0, 0]).astype(float)]
        self.twist_array_P = np.array([0, 0, 0, 0]).astype(float)
        self.twist_array_D = np.array([0, 0, 0, 0]).astype(float)
        self.twist_array_I = np.array([0, 0, 0, 0]).astype(float)
        self.filter_index= 0

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

        # self.create_timer(
        #     timer_period_sec=5,
        #     callback=self.test_pid,
        #     callback_group=MutuallyExclusiveCallbackGroup()
        # )

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
            qos_profile=QoSProfile(depth=1),
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
                timeout=rclpy.time.Duration(seconds=0.1)
            )
            self.subscribe_goal(self.goal_pose)
        except:
            pass


    def subscribe_goal(self, msg:PoseStamped):

        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        secunds = sec + nanosec / 1000000000
        self.delta_time = secunds - self.secunds 
        self.secunds = sec + nanosec / 1000000000

        self.goal_pose = msg

        goal_pose: PoseStamped = tf2_geometry_msgs.do_transform_pose_stamped(self.goal_pose, self.transform)

        x = goal_pose.pose.position.x * 1.0
        y = goal_pose.pose.position.y * 1.0
        z = goal_pose.pose.position.z * 1.0


        psi = self.pose_to_psi(goal_pose.pose) / math.pi
        # psi = 0.0

        self.twist_array_filter[self.filter_index] = np.array([x, y, z, psi]).astype(float)
        self.filter_index += 1
        if self.filter_index >= 3:
            self.filter_index = 0

        self.twist_array = np.sum(self.twist_array_filter, axis=0) / 3

        self.twist_array_P = self.twist_array
        # self.twist_array_P = np.array([x, y, z, psi]).astype(float)
        # self.twist_array_D = (self.twist_array_1 - self.twist_array) * self.delta_time
        # self.twist_array_I = self.twist_array + self.twist_array_I * self.delta_time

        # for i, value in enumerate(self.twist_array_I):
        #     if value > 0.2:
        #         self.twist_array_I[i] = 0.2
        #     elif value < -0.2:
        #         self.twist_array_I[i] = -0.2

        self.twist_array_PID = self.twist_array_P * 0.15# + self.twist_array_D * 0.04# + self.twist_array_I * 0.12 

        for i, value in enumerate(self.twist_array_PID):
            if value > 0.4:
                self.twist_array_PID[i] = 0.4
            elif value < -0.4:
                self.twist_array_PID[i] = -0.4
        
        # self.get_logger().info(f"{self.twist_array_PID}")
        message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = msg.header.stamp

        message.twist.linear.x = self.twist_array_PID[0]
        message.twist.linear.y = self.twist_array_PID[1]
        message.twist.linear.z = self.twist_array_PID[2]

        message.twist.angular.z = self.twist_array_PID[3]

        self.publisher_cmd_vel.publish(message)

        self.twist_array_1 = self.twist_array

    def pose_to_psi(self, pose: Pose):
        return math.atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1 - 2*(pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z))
    
    def subscribe_base_link(self, msg: Odometry):
        odometry_pose = msg.pose.pose

    def test_pid(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "odom"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.x_test = self.x_test * -1
        self.z_test = self.z_test * -1
        self.goal_pose.pose.position.x = self.x_test
        self.goal_pose.pose.position.z = 0.89 #self.z_test




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