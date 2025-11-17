import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge



class API(Node):
    def __init__(self):
        super().__init__("API")

        third_party = os.path.join(get_package_share_directory('tello'), 'third_party')

        if third_party not in sys.path:
            sys.path.insert(0, third_party)

        from djitellopy import Tello

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
        
        self.publisher_velocity = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            topic="control/twist0_velocity",
            qos_profile=QoSProfile(depth=10),
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
        
        #publishing image to rviz
        self.create_timer(
            timer_period_sec=0.06,
            callback=self.publish_image,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #logging status of tello
        self.create_timer(
            timer_period_sec=15,
            callback=self.tello_status_report,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #ping frame from tello
        self.create_timer(
            timer_period_sec=0.06,
            callback=self.get_frame,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        #ping velocity from tello
        self.create_timer(
            timer_period_sec=0.01,
            callback=self.get_velocity,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # publish velosity
        self.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_velocity,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # publish camera info
        self.timer = self.create_timer(
            timer_period_sec=1, 
            callback=self.publish_camera_info,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def get_frame(self):
        frame = self.frame_backend.frame
        if frame is None:
            self.get_logger().warning("Frame object returned None")
        else:
            frame = np.array(frame)
            self.frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # self.get_logger().info(f"{self.frame.shape}")
    
    def publish_image(self):
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

    def get_velocity(self):
        x = self.tello.get_speed_x() / 100
        y = self.tello.get_speed_y() / 100
        z = self.tello.get_speed_z() / 100

        self.velocity_list.append(np.array([[x, y, z], [0, 0, 0]]).T)

    def publish_velocity(self):
        message = TwistWithCovarianceStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f"{self.velocity_list[0].shape}")
        if len(self.velocity_list):
            velocity = np.sum(self.velocity_list, axis=0) / len(self.velocity_list)
            message.twist.twist.linear.x = velocity[0, 0]
            message.twist.twist.linear.y = velocity[1, 0]
            message.twist.twist.linear.z = -velocity[2, 0]
            # message.twist.twist.angular.x = velocity[0, 1]
            # message.twist.twist.angular.y = velocity[1, 1]
            # message.twist.twist.angular.z = velocity[2, 1]
        
        message.twist.covariance = np.array([
            [1.0, 0.1, 0.1, 0.0,  0.0,  0.1],
            [0.1, 1.0, 0.1, 0.0,  0.0,  0.1],
            [0.1, 0.1, 1.0, 0.0,  0.0,  0.1],
            [0.0, 0.0, 0.0, 0.01, 0.0,  0.0],
            [0.0, 0.0, 0.0, 0.0,  0.01, 0.0],
            [0.1, 0.1, 0.1, 0.0,  0.0,  1.0],
        ]).flatten().tolist()

        self.publisher_velocity.publish(message)

    def subscribe_twist(self, msg: TwistStamped):

        # self.get_logger().info("Twist")

        left_right = int(msg.twist.linear.y * -100)
        forward_backward = int(msg.twist.linear.x * 100)
        up_down = int(msg.twist.linear.z * 100)
        yaw = int(msg.twist.angular.z * -100)

        self.tello.send_rc_control(
            left_right_velocity=left_right,
            forward_backward_velocity=forward_backward,
            up_down_velocity=up_down,
            yaw_velocity=yaw
        )

    def takeoff_land(self, msg: String):
        self.get_logger().info(f"{msg.data}")
        if self.flying:
            self.tello.land()
            self.flying = False
        else:
            self.tello.takeoff()
            self.flying = True

    def e_stop(self, msg):
        self.tello.emergency() 

    def shutdown(self):
        pass
        self.tello.emergency() 
        self.tello.streamoff()

    def publish_camera_info(self):
        message = CameraInfo()
        message.header.frame_id = 'camera'
        message.header.stamp = self.get_clock().now().to_msg()

        message.height = 720
        message.width = 960
        message.distortion_model = 'plumb_bob'
        message.d = [0, 0, 0, 0, 0]
        message.k = [552, 0, 480, 0, 552, 360, 0, 0, 1]
        message.r = [1,0,0,0,1,0,0,0,1]
        message.p = [552,0,480,0,0,552,360,0,0,0,1,0]

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