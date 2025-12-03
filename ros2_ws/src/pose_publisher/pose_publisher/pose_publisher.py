#!/usr/bin/env python3

import math
import re
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.qos import QoSProfile




class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Topics / frames
        self.GOAL_POSE_TOPIC      = 'goal_pose'
        self.PID_ERROR_TOPIC      = 'pid_error'
        self.CMD                  = 'cmd_vel'
        self.JOY_BUTTON_TOPIC     = 'next_waypoint'
        self.LAND_TOPIC           = 'arm'
        self.FRAME_ID             = 'odom'

        # Publishing rate (seconds)
        self.PUBLISH_PERIOD       = 0.5

        # Stability logic
        self.STABLE_SAMPLES       = 20        # how many readings must be below thresholds

        self.THRESH_X             = 0.0
        self.THRESH_Y             = 0.0
        self.THRESH_Z             = 0.0
        self.THRESH_TOTAL         = 0.0

        self.THRESH_ANGLE = 0.25

        START_WAYPOINT = 0

        # Waypoint YAML location
        self.WAYPOINT_PACKAGE     = 'drone'               # ROS2 package containing the YAML
        self.WAYPOINT_FILE_REL    = 'params/waypoints.yaml'  # path inside that package

        self.THRESH_X = self.get_parameter_or("thresh_x", self.THRESH_X).value
        self.THRESH_Y = self.get_parameter_or("thresh_y", self.THRESH_Y).value
        self.THRESH_Z = self.get_parameter_or("thresh_y", self.THRESH_Z).value
        self.THRESH_TOTAL = self.get_parameter_or("thresh_total", self.THRESH_TOTAL).value

        self.markers_id_list = self.get_parameter("waypoint").value
        
        self.waypoint_msg_list: list[PoseStamped] = []
        frame_id = self.get_parameter_or("header.frame_id", self.FRAME_ID).value

        waypoint_list: list[PoseStamped] = []
        for id in self.markers_id_list:
            waypoint = PoseStamped()
            waypoint = PoseStamped()
            waypoint.header.frame_id = frame_id
            pose = Pose()
            pose.position.x = self.get_parameter_or(f"waypoint_{id}.pose.position.x", 0.0).value
            pose.position.y = self.get_parameter_or(f"waypoint_{id}.pose.position.y", 0.0).value
            pose.position.z = self.get_parameter_or(f"waypoint_{id}.pose.position.z", 0.0).value
            pose.orientation.x = self.get_parameter_or(f"waypoint_{id}.pose.orientation.x", 0.0).value
            pose.orientation.y = self.get_parameter_or(f"waypoint_{id}.pose.orientation.y", 0.0).value
            pose.orientation.z = self.get_parameter_or(f"waypoint_{id}.pose.orientation.z", 0.0).value
            pose.orientation.w = self.get_parameter_or(f"waypoint_{id}.pose.orientation.w", 0.0).value
            waypoint.pose = pose
            waypoint_list.append(waypoint)

        # self.get_logger().error(f'Waypoints: {waypoint_list}')


        self.waypoint_msg_list = waypoint_list


        # Publisher for goal pose for the controller
        self.publisher_ = self.create_publisher(PoseStamped, self.GOAL_POSE_TOPIC, 10)

        self.current_index = START_WAYPOINT

        # Keep track of last N "is stable" results
        self.stable_flags = deque(maxlen=self.STABLE_SAMPLES)

        # Subscriber for pid_error
        self.pid_sub = self.create_subscription(
            PoseStamped,
            self.PID_ERROR_TOPIC,
            self.pid_error_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            String,
            self.JOY_BUTTON_TOPIC,
            self.manual_next_waypoint,
            1
        )

        self.land_pub = self.create_publisher(
            msg_type=String,
            topic=self.LAND_TOPIC,
            qos_profile=QoSProfile(depth=10),
        )
        # Timer for publishing the current goal pose
        self.timer = self.create_timer(self.PUBLISH_PERIOD, self.timer_callback)

    def pose_to_psi(self, pose: Pose):
        return math.atan2(2*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y), 1 - 2*(pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z))
    

    # Callbacks

    def pid_error_callback(self, msg: PoseStamped):
        """Store whether this error sample is within all thresholds."""
        err_x = msg.pose.position.x
        err_y = msg.pose.position.y
        err_z = msg.pose.position.z

        # Per-axis
        within_axes = (
            abs(err_x) < self.THRESH_X and
            abs(err_y) < self.THRESH_Y and
            abs(err_z) < self.THRESH_Z
        )

        # Total distance
        dist = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        within_total = dist < self.THRESH_TOTAL

        within_angle = self.THRESH_ANGLE > self.pose_to_psi(msg.pose)

        is_stable = within_axes and within_total and within_angle

        if is_stable:
            self.stable_flags.append(is_stable)

        # Check if the last N samples are all stable
        if (len(self.stable_flags) >= self.STABLE_SAMPLES):
            if self.current_index > 0:
                self.advance_waypoint()
            # pass
    

    def manual_next_waypoint(self, msg: String):
        self.advance_waypoint()

    def timer_callback(self):
        """Publish current waypoint pose repeatedly."""
        msg = self.waypoint_msg_list[self.current_index]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        

    def advance_waypoint(self):
        """Advance to the next waypoint when stable, if any."""
        self.get_logger().info(f'Nr poses: {len(self.waypoint_msg_list)}')
        if self.current_index < len(self.waypoint_msg_list) - 1:
            self.current_index += 1
            self.stable_flags.clear()  # reset stability history for the new target
            self.get_logger().info(f'Advancing to waypoint {self.current_index + 1}/{len(self.waypoint_msg_list)}')
        else:
            self.get_logger().info('Last waypoint reached; not advancing further.')
            msg = String()
            msg.data = "Last Waypoint, landing drone"
            


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
