#!/usr/bin/env python3

import math
import re
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml


# Topics / frames
GOAL_POSE_TOPIC      = '/goal_pose'
PID_ERROR_TOPIC      = '/pid_error'
JOY_BUTTON_TOPIC     = '/next_waypoint'
FRAME_ID             = 'odom'

# Publishing rate (seconds)
PUBLISH_PERIOD       = 0.5

# Stability logic
STABLE_SAMPLES       = 5        # how many readings must be below thresholds
THRESH_X             = 0.15
THRESH_Y             = 0.15
THRESH_Z             = 0.15
THRESH_TOTAL         = 0.25

# Waypoint YAML location
WAYPOINT_PACKAGE     = 'drone'               # ROS2 package containing the YAML
WAYPOINT_FILE_REL    = 'params/waypoints.yaml'  # path inside that package


class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        THRESH_X = self.get_parameter_or("thresh_x", THRESH_X).value
        THRESH_Y = self.get_parameter_or("thresh_x", THRESH_Y).value
        THRESH_Z = self.get_parameter_or("thresh_x", THRESH_Z).value
        THRESH_TOTAL = self.get_parameter_or("thresh_x", THRESH_TOTAL).value

        self.markers_id_list = self.get_parameter("waypoint").value
        
        self.waypoint_msg_list: list[PoseStamped] = []
        frame_id = self.get_parameter_or("header.frame_id", FRAME_ID).value

        waypoint_list: list[PoseStamped] = []
        for id in self.markers_id_list:
            waypoint = PoseStamped()
            waypoint = PoseStamped()
            waypoint.header.frame_id = frame_id
            pose = Pose()
            pose.position.x = self.get_parameter_or(f"marker_{id}.pose.position.x", 0.0).value
            pose.position.y = self.get_parameter_or(f"marker_{id}.pose.position.y", 0.0).value
            pose.position.z = self.get_parameter_or(f"marker_{id}.pose.position.z", 0.0).value
            pose.orientation.x = self.get_parameter_or(f"marker_{id}.pose.orientation.x", 0.0).value
            pose.orientation.y = self.get_parameter_or(f"marker_{id}.pose.orientation.y", 0.0).value
            pose.orientation.z = self.get_parameter_or(f"marker_{id}.pose.orientation.z", 0.0).value
            pose.orientation.w = self.get_parameter_or(f"marker_{id}.pose.orientation.w", 0.0).value
            waypoint.pose = pose
            waypoint_list.append(waypoint)

        self.waypoint_msg_list = waypoint_list


        # Publisher for goal pose for the controller
        self.publisher_ = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)

        # # Load waypoints from YAML
        # self.poses = self.load_waypoints()
        # if not self.poses:
        #     self.get_logger().error('No waypoints loaded. Node will still run but publish nothing useful.')
        self.current_index = 0

        # Keep track of last N "is stable" results
        self.stable_flags = deque(maxlen=STABLE_SAMPLES)

        # Subscriber for pid_error
        self.pid_sub = self.create_subscription(
            PoseStamped,
            PID_ERROR_TOPIC,
            self.pid_error_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            String,
            JOY_BUTTON_TOPIC,
            self.manual_next_waypoint,
            1
        )

        # Timer for publishing the current goal pose
        self.timer = self.create_timer(PUBLISH_PERIOD, self.timer_callback)

    # YAML loading

    def load_waypoints(self):
        """Load waypoints from YAML and return a sorted list of dicts."""
        try:
            pkg_share = get_package_share_directory(WAYPOINT_PACKAGE)
        except Exception as e:
            self.get_logger().error(f'Could not find package "{WAYPOINT_PACKAGE}": {e}')
            return []

        yaml_path = Path(pkg_share) / WAYPOINT_FILE_REL
        if not yaml_path.is_file():
            self.get_logger().error(f'Waypoint YAML not found: {yaml_path}')
            return []

        try:
            with yaml_path.open('r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from {yaml_path}: {e}')
            return []

        def key_num(name):
            m = re.search(r'(\d+)$', str(name))
            return int(m.group(1)) if m else 0

        poses = []
        for key, node in sorted(data.items(), key=lambda kv: key_num(kv[0])):
            if not isinstance(node, dict):
                continue
            pose_node = node.get('pose', {})
            if not isinstance(pose_node, dict):
                continue

            pos = pose_node.get('position', {}) or {}
            ori = pose_node.get('orientation', {}) or {}

            try:
                x = float(pos.get('x', 0.0))
                y = float(pos.get('y', 0.0))
                z = float(pos.get('z', 0.0))
                yaw_deg = float(ori.get('yaw', 0.0))
            except (TypeError, ValueError) as e:
                self.get_logger().warn(f'Invalid values in waypoint "{key}": {e}')
                continue

            poses.append({'x': x, 'y': y, 'z': z, 'yaw_deg': yaw_deg})

        self.get_logger().info(f'Loaded {len(poses)} waypoints from {yaml_path}')
        return poses

    # Callbacks

    def pid_error_callback(self, msg: PoseStamped):
        """Store whether this error sample is within all thresholds."""
        err_x = float(msg.pose.position.x)
        err_y = float(msg.pose.position.y)
        err_z = float(msg.pose.position.z)

        # Per-axis
        within_axes = (
            abs(err_x) < THRESH_X and
            abs(err_y) < THRESH_Y and
            abs(err_z) < THRESH_Z
        )

        # Total distance
        dist = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        within_total = dist < THRESH_TOTAL

        is_stable = within_axes and within_total
        self.stable_flags.append(is_stable)

        # Debugging
        # self.get_logger().info(
        #     f'pid_error: ({err_x:.3f}, {err_y:.3f}, {err_z:.3f}), '
        #     f'dist={dist:.3f}, stable={is_stable}'
        # )

        # Check if the last N samples are all stable
        if (
            len(self.stable_flags) == STABLE_SAMPLES and
            all(self.stable_flags)
        ):
            self.advance_waypoint()
    

    def manual_next_waypoint(self, msg: String):
        self.advance_waypoint()

    def timer_callback(self):
        """Publish current waypoint pose repeatedly."""
        # if not self.poses:
        #     # nothing to do
        #     return

        # # Index to last waypoint once finished
        # idx = min(self.current_index, len(self.poses) - 1)
        # wp = self.poses[idx]

        # msg = PoseStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = FRAME_ID

        # msg.pose.position.x = wp['x']
        # msg.pose.position.y = wp['y']
        # msg.pose.position.z = wp['z']

        # # Convert yaw [deg] to quaternion
        # yaw_rad = math.radians(wp['yaw_deg'])
        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        # msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

        msg = self.waypoint_msg_list[self.current_index]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        

    def advance_waypoint(self):
        """Advance to the next waypoint when stable, if any."""
        if self.current_index < len(self.poses) - 1:
            self.current_index += 1
            self.stable_flags.clear()  # reset stability history for the new target
            self.get_logger().info(f'Advancing to waypoint {self.current_index + 1}/{len(self.poses)}')
        else:
            self.get_logger().info('Last waypoint reached; not advancing further.')


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
