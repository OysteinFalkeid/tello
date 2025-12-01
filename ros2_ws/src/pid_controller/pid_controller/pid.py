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
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.position.x = 0.0

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

        ##########
        self.z_low = 0.5
        self.z_high = 0.3
        self.z_toggle = False

        # Gains per axis: [x, y, z, yaw]
        self.Kp = np.array([0.15, 0.15, 0.5, 0.8], dtype=float)
        self.Kd = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
        # self.Kp = np.array([0.00, 0.00, 1.5, 0.0], dtype=float)
        # self.Kd = np.array([0.00, 0.00, 0.2, 0.0], dtype=float)
        self.Ki = np.array([0.15, 0.15, 0.25, 0.0], dtype=float)  # start with no Ki in x,y,yaw 
        # z between 110 and 130 isntead of 70 to 90

        self.error_prev = np.zeros(4, dtype=float)
        self.error_int  = np.zeros(4, dtype=float)

        self.u_max = np.array([0.30, 0.30, 0.40, 0.2], dtype=float)  # cmd_vel limits
        ##########

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

        self.pid_error_pub = self.create_publisher(
            PoseStamped,
            "pid_error",  # this becomes /tello/control/pid_error if the node has that namespace
            QoSProfile(depth=10),
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
        self.pid_error_pub.publish(goal_pose)

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

        # error in base_link frame
        error = self.twist_array  # [ex, ey, ez, epsi]

        # -----------------------
        # error in base_link frame
        error = self.twist_array  # [ex, ey, ez, epsi]

        deadband = np.array([0.05, 0.05, 0.05, 0.03], dtype=float)
        for i in range(4):
            if abs(error[i]) < deadband[i]:
                error[i] = 0.0

        # # publish error as PoseStamped
        # err_msg = PoseStamped()
        # err_msg.header.stamp = msg.header.stamp
        # err_msg.header.frame_id = "base_link"

        # err_msg.pose.position.x = float(error[0])
        # err_msg.pose.position.y = float(error[1])
        # err_msg.pose.position.z = float(error[2])

        # # stash yaw error in orientation.z (or wherever you like)
        # err_msg.pose.orientation.z = float(error[3])
        # err_msg.pose.orientation.w = 1.0

        # self.pid_error_pub.publish(err_msg)


        # time step
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        t_now = sec + nanosec * 1e-9


        if not hasattr(self, "t_prev"):
            self.t_prev = t_now
            self.secunds = t_now
            return  # skip first loop so dt is valid

        dt = t_now - self.t_prev
        self.t_prev = t_now
        if dt <= 1e-5:
            dt = 1e-3  # avoid crazy derivative

        # --- PID terms ---

        # P
        P = self.Kp * error

        # D  (derivative of error: (e - e_prev)/dt)
        D = self.Kd * ((error - self.error_prev) * dt)
        self.error_prev = error.copy()

        # I with anti-windup
        self.error_int += error * dt
        # clamp integral
        # self.error_int = np.clip(self.error_int, -0.2, 0.2)
        I = self.Ki * self.error_int

        I = np.clip(I, -0.008, 0.008)

        u = P + D + I

        # saturate per axis
        u = np.clip(u, -self.u_max, self.u_max)

        # build message
        message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = msg.header.stamp

        message.twist.linear.x  = float(u[0])
        message.twist.linear.y  = float(u[1])
        message.twist.linear.z  = float(u[2])
        message.twist.angular.z = float(u[3])

        self.publisher_cmd_vel.publish(message)


        # self.twist_array_P = self.twist_array
        # # self.twist_array_P = np.array([x, y, z, psi]).astype(float)
        # # self.twist_array_D = (self.twist_array_1 - self.twist_array) * self.delta_time
        # # self.twist_array_I = self.twist_array + self.twist_array_I * self.delta_time

        # # for i, value in enumerate(self.twist_array_I):
        # #     if value > 0.2:
        # #         self.twist_array_I[i] = 0.2
        # #     elif value < -0.2:
        # #         self.twist_array_I[i] = -0.2

        # self.twist_array_PID = self.twist_array_P * 0.15 * 2 # + self.twist_array_D * 0.04# + self.twist_array_I * 0.12 

        # for i, value in enumerate(self.twist_array_PID):
        #     if value > 0.8:
        #         self.twist_array_PID[i] = 0.8
        #     elif value < -0.8:
        #         self.twist_array_PID[i] = -0.8
        
        # # self.get_logger().info(f"{self.twist_array_PID}")
        # message = TwistStamped()
        # message.header.frame_id = "base_link"
        # message.header.stamp = msg.header.stamp

        # message.twist.linear.x = self.twist_array_PID[0]
        # message.twist.linear.y = self.twist_array_PID[1]
        # message.twist.linear.z = self.twist_array_PID[2]*2

        # message.twist.angular.z = self.twist_array_PID[3]*2

        # self.publisher_cmd_vel.publish(message)

        # self.twist_array_1 = self.twist_array

    def pose_to_psi(self, pose: Pose):
        return math.atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1 - 2*(pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z))
    
    def subscribe_base_link(self, msg: Odometry):
        odometry_pose = msg.pose.pose

    # def test_pid(self):
    #     self.goal_pose = PoseStamped()
    #     self.goal_pose.header.frame_id = "odom"
    #     self.goal_pose.header.stamp = self.get_clock().now().to_msg()
    #     self.x_test = self.x_test * -1
    #     self.z_test = self.z_test * -1
    #     self.goal_pose.pose.position.x = self.x_test
    #     self.goal_pose.pose.position.z = 1.0 #self.z_test

    def test_pid(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "odom"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()

        if self.z_toggle:
            z_goal = self.z_low   # 0.8 m
        else:
            z_goal = self.z_high  # 0.9 m

        self.z_toggle = not self.z_toggle

        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = z_goal
        self.goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f"New Z step goal: {z_goal:.2f} m")




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