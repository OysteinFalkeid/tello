import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import subprocess

def connect_to_wifi(ssid: str, logger):
    try:
        # Try connecting (works for open or known networks)
        result = subprocess.run(
            ["nmcli", "device", "wifi", "connect", ssid],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            logger(f"Connected to: {ssid}")
            return True
        else:
            logger(f"Failed to connect: {result.stderr}")
            return False

    except Exception as e:
        logger(f"Error: {e}")
        return False
    

class API(Node):
    def __init__(self):
        super().__init__("API")

        self.takeoff = False

        third_party = os.path.join(get_package_share_directory('tello'), 'third_party')

        if third_party not in sys.path:
            sys.path.insert(0, third_party)

        from djitellopy import Tello

        while connect_to_wifi("", self.get_logger().info):
            pass

        self.tello = Tello()

        self.tello.connect()
        self.tello.streamon()
        self.frame_backend = self.tello.get_frame_read()

        self.flying = False
        
        self.bridge = CvBridge()

        self.frame = np.array([
            [[255, 255, 255],   [0, 0, 0],          [255, 255, 255] ],
            [[0, 0, 0],         [255, 255, 255],    [0, 0, 0]       ],
            [[255, 255, 255],   [0, 0, 0],          [255, 255, 255] ],
        ]).astype(np.uint8)

        self.pose = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]).T

        self.covariance = np.array([
            [0.04,  0.01,   0.01,   1e-6,    1e-6,    0.002 ],
            [0.01,  0.09,   1e-6,    1e-6,    1e-6,    0.003 ],
            [0.01,  0.01,   0.04,   1e-6,    1e-6,    0.001 ],
            [1e-6,   1e-6,    1e-6,    1e-6,   1e-6,    1e-6   ],
            [1e-6,   1e-6,    1e-6,    1e-6,    1e-6,   1e-6   ],
            [0.002, 0.003,  0.001,  1e-6,    1e-6,    0.0025]
        ])

        self.velocity_list = [np.array([[0.0,0.0,0.0],[0.0,0.0,0.0]]).T]

        self.publisher_image = self.create_publisher(
            msg_type=Image,
            topic="camera/image",
            qos_profile=QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.publisher_camera_info = self.create_publisher(
            msg_type=CameraInfo,
            topic="camera/camera_info",
            qos_profile=QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.publisher_acceleration = self.create_publisher(
            msg_type=Imu,
            topic="control/imu0_acceleration",
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.publisher_velocity = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            # msg_type=TwistStamped,
            topic="control/twist0_velocity",
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.publisher_hight = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic="control/pose1_hight",
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # Twist
        self.create_subscription(
            msg_type=TwistStamped,
            topic="/tello/control/cmd_vel",
            callback=self.subscribe_twist,
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()

        )

        # Takeof/Land
        self.create_subscription(
            msg_type=String,
            topic="/tello/control/arm",
            callback=self.takeoff_land,
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # E stop
        self.create_subscription(
            msg_type=String,
            topic="/tello/control/e_stop",
            callback=self.e_stop,
            qos_profile=QoSProfile(depth=1),
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #logging status of tello
        self.create_timer(
            timer_period_sec=1,
            callback=self.tello_status_report,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #ping frame from tello
        self.create_timer(
            timer_period_sec=0.08,
            callback=self.get_frame,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # #ping acceleration from tello
        # self.create_timer(
        #     timer_period_sec=0.01,
        #     callback=self.get_acceleration,
        #     callback_group=MutuallyExclusiveCallbackGroup()
        # )

        # #ping velocity from tello
        # self.create_timer(
        #     timer_period_sec=0.01,
        #     callback=self.get_velocity,
        #     callback_group=MutuallyExclusiveCallbackGroup()
        # )

        #ping hight from tello
        self.create_timer(
            timer_period_sec=0.03,
            callback=self.get_hight,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # publish camera info
        self.create_timer(
            timer_period_sec=1, 
            callback=self.publish_camera_info,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def safe_check_frame(self, frame: np.ndarray):
        if frame is None:
            return False, "None frame"

        # if not isinstance(frame, np.ndarray):
        #     return False, "Not a numpy array"

        if frame.dtype != np.uint8:
            return False, f"Invalid dtype {frame.dtype}"

        if frame.ndim not in (2, 3):
            return False, f"Invalid ndim {frame.ndim}"

        if frame.ndim == 3 and frame.shape[2] not in (1, 3, 4):
            return False, f"Invalid channel count {frame.shape}"

        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)

        return True, ""

    def get_frame(self):
        frame = np.array(self.frame_backend.frame)
        frame_status, result = self.safe_check_frame(frame)
        if frame_status:
            frame = np.array(frame)
            self.frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # self.get_logger().info(f"{self.frame.shape}")
        else:
            self.get_logger().info(result)
        header = Header()
        header.frame_id = "camera"
        header.stamp = self.get_clock().now().to_msg()
        message = self.bridge.cv2_to_imgmsg(cvim=self.frame, encoding='bgr8', header=header)
        self.publisher_image.publish(message)

    def tello_status_report(self):
        self.get_logger().info("Tello status report --------------------------")
        self.battery = self.tello.get_battery()
        self.get_logger().info(F"Battery {self.battery}%")
        self.height = self.tello.get_height()
        self.get_logger().info(F"Height {self.height}")
        # self.barometer = self.tello.get_barometer()
        # self.get_logger().info(F"Barometer {self.barometer/100}m")
        self.temperature = self.tello.get_temperature()
        self.get_logger().info(F"Temperature {self.temperature}C")
        self.flight_time = self.tello.get_flight_time()
        self.get_logger().info(F"Flight_time {self.flight_time}")
        self.get_logger().info("---------------------------------------------")

    def get_acceleration(self):
        x = self.tello.get_acceleration_x() / 100.0
        y = self.tello.get_acceleration_y() / -100.0
        z = self.tello.get_acceleration_z() / 100.0

        message = Imu()
        message.header.frame_id = "base_link"
        message.header.stamp = self.get_clock().now().to_msg()
        message.linear_acceleration.x = x
        message.linear_acceleration.y = y
        message.linear_acceleration.z = z

        message.linear_acceleration_covariance = np.array([
            [1.0, 0.1, 0.1, 0.0,  0.0,  0.0],
            [0.1, 1.0, 0.1, 0.0,  0.0,  0.0],
            [0.1, 0.1, 1.0, 0.0,  0.0,  0.0],
            [0.0, 0.0, 0.0, 0.01,  0.0,  0.0],
            [0.0, 0.0, 0.0, 0.0,  0.01,  0.0],
            [0.0, 0.0, 0.0, 0.0,  0.0,  0.01],
        ]).flatten().tolist()

        self.publisher_acceleration.publish(message)

    def get_velocity(self):
        x = self.tello.get_speed_x() / 100.0
        y = self.tello.get_speed_y() / -100.0
        z = self.tello.get_speed_z() / 100.0
        
        # self.velocity_list.append(np.array([[x, -y, z], [0, 0, 0]]).astype(float).T)


        # velocity = np.array([[x, -y, z], [0, 0, 0]]).astype(float).T

        # message = TwistWithCovarianceStamped()
        # message.header.frame_id = "base_link"
        # message.header.stamp = self.get_clock().now().to_msg()

        # message.twist.twist.linear.x = velocity[0, 0]
        # message.twist.twist.linear.y = velocity[1, 0]
        # message.twist.twist.linear.z = velocity[2, 0]
        # # message.twist.twist.angular.x = velocity[0, 1]
        # # message.twist.twist.angular.y = velocity[1, 1]
        # # message.twist.twist.angular.z = velocity[2, 1]        

        message = TwistWithCovarianceStamped()
        # message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = self.get_clock().now().to_msg()

        message.twist.twist.linear.x = x
        message.twist.twist.linear.y = y
        message.twist.twist.linear.z = z
        
        message.twist.covariance = np.array([
            [1.0, 0.1, 0.1, 0.0,  0.0,  0.1],
            [0.1, 1.0, 0.1, 0.0,  0.0,  0.1],
            [0.1, 0.1, 1.0, 0.0,  0.0,  0.1],
            [0.0, 0.0, 0.0, 0.01, 0.0,  0.0],
            [0.0, 0.0, 0.0, 0.0,  0.01, 0.0],
            [0.1, 0.1, 0.1, 0.0,  0.0,  1.0],
        ]).flatten().tolist()        
        
        # message.twist.linear.x = x
        # message.twist.linear.y = y
        # message.twist.linear.z = z
        
        if self.takeoff:
            self.publisher_velocity.publish(message)

    def get_hight(self):
        # z = self.tello.get_height() / 100.0
        z = self.tello.get_distance_tof() / 100.0
        if z > 0.01:
            message = PoseWithCovarianceStamped()
            message.header.frame_id = "odom"
            message.header.stamp = self.get_clock().now().to_msg()

            message.pose.pose.position.z = z

            message.pose.covariance = (np.array([
                [1.0, 0.0, 0.0, 0.0,  0.0,  0.0],
                [0.0, 1.0, 0.0, 0.0,  0.0,  0.0],
                [0.0, 0.0, 1.0, 0.0,  0.0,  0.0],
                [0.0, 0.0, 0.0, 1.0,  0.0,  0.0],
                [0.0, 0.0, 0.0, 0.0,  1.0,  0.0],
                [0.0, 0.0, 0.0, 0.0,  0.0,  1.0],
            ]) * 2).flatten().tolist()

            self.publisher_hight.publish(message)

    def subscribe_twist(self, msg: TwistStamped):

        # self.get_logger().info("Twist")

        left_right = int(msg.twist.linear.y * -100)
        forward_backward = int(msg.twist.linear.x * 100)
        up_down = int(msg.twist.linear.z * 100)
        yaw = int(msg.twist.angular.z * -100 / 1.2)

        if self.takeoff:
            self.tello.send_rc_control(
                left_right_velocity=left_right,
                forward_backward_velocity=forward_backward,
                up_down_velocity=up_down,
                yaw_velocity=yaw
            )

    def takeoff_land(self, msg: String):
        self.get_logger().info(f"{msg.data}")
        if self.tello.is_flying:
            self.takeoff = False
            self.tello.land()
            self.flying = False
        else:
            self.tello.takeoff()
            self.flying = True
            self.takeoff = True

    def e_stop(self, msg: String):
        self.shutdown()
        self.get_logger().warning(f"{msg.data}")
        raise KeyboardInterrupt

    def shutdown(self):
        self.tello.emergency() 
        self.tello.streamoff()
        self.tello.reboot()

    def publish_camera_info(self):
        message = CameraInfo()
        message.header.frame_id = 'camera'
        message.header.stamp = self.get_clock().now().to_msg()

        message.height = 720
        message.width = 960
        message.distortion_model = 'plumb_bob'
        message.d = [-0.016272, 0.093492, 0.000093, 0.002999, 0.000000]
        message.k = [929.562627, 0.000000, 487.474037, 0.000000, 928.604856, 363.165223, 0.000000, 0.000000, 1.000000]
        message.r = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
        message.p = [937.878723, 0.000000, 489.753885, 0.000000, 0.000000, 939.156738, 363.172139, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

        self.publisher_camera_info.publish(message)


def main():
    rclpy.init() 
    api = API()

    # Use a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=8)
    if executor.add_node(api):
        
        try:
            executor.spin()
        finally:
            api.shutdown()
            api.destroy_node()
            rclpy.shutdown()
    else:
            api.shutdown()
            api.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()