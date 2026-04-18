#!/usr/bin/env python3
from __future__ import annotations
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from booster_interface.msg import Odometer, LowState, BatteryState as BoosterBatteryState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# cv2, numpy, cv_bridge are imported lazily in on_configure() to avoid
# blocking the node from reaching 'unconfigured' before the state machine connects.

class RobotRead(LifecycleNode):

    def __init__(self):
        super().__init__('robot_read_node')

        # Parameters must be declared in the constructor.
        self.declare_parameter('prefix', '')
        self.prefix = '' 

        # Initialize all ROS communicators to None.
        self.imu_subscriber = None
        self.stereo_rgb_subscriber = None
        self.stereo_depth_subscriber = None
        self.stereo_info_subscriber = None
        self.stereo_visual_subscriber = None
        self.battery_subscriber = None

        self.tf_broadcaster = None

        self.odom_pub = None
        self.imu_pub = None
        self.battery_pub = None
        self.stereo_rgb_pub = None
        self.stereo_depth_pub = None
        self.stereo_info_pub = None
        self.stereo_visual_pub = None

        self.bridge = None
        self._last_rgb_time = 0.0
        self._rgb_min_interval = 1.0 / 10.0  # max 10 fps for CPU-heavy NV12 decode
        self._last_depth_time = 0.0
        self._depth_min_interval = 1.0 / 10.0  # max 10 fps, sufficient for Nav2 costmaps
        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        # cv2/numpy/cv_bridge are heavy C-extensions — deferred until configure so
        # the node advertises its lifecycle services before spending time on imports.
        import numpy as np
        import cv2
        from cv_bridge import CvBridge
        self._np = np
        self._cv2 = cv2
        self.bridge = CvBridge()

        # Get the parameter value now that the node is being configured.
        self.prefix = self.get_parameter('prefix').value
        self.get_logger().info(f"Using prefix: '{self.prefix}'")
        
        stereo_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.odom_subscriber = self.create_subscription(Odometer, '/odometer_state', self.odom_subscriber_callback, 10)
        self.imu_subscriber = self.create_subscription(LowState, '/low_state', self.imu_subscriber_callback, 10)
        self.stereo_rgb_subscriber = self.create_subscription(Image, '/StereoNetNode/rectified_image', self.stereo_rgb_callback, stereo_qos)
        self.stereo_depth_subscriber = self.create_subscription(Image, '/StereoNetNode/stereonet_depth', self.stereo_depth_callback, stereo_qos)
        self.stereo_info_subscriber = self.create_subscription(CameraInfo, '/StereoNetNode/stereonet_depth/camera_info', self.stereo_info_callback, stereo_qos)
        self.stereo_visual_subscriber = self.create_subscription(Image, '/StereoNetNode/stereonet_visual', self.stereo_visual_callback, stereo_qos)
        self.battery_subscriber = self.create_subscription(BoosterBatteryState, '/battery_state', self.battery_callback, 1)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 1)
        self.stereo_rgb_pub = self.create_publisher(Image, '/rgb/image', stereo_qos)
        self.stereo_depth_pub = self.create_publisher(Image, '/depth/image', stereo_qos)
        self.stereo_info_pub = self.create_publisher(CameraInfo, '/rgb/camera_info', stereo_qos)
        self.stereo_visual_pub = self.create_publisher(Image, '/depth/visual', stereo_qos)

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_activate() is called.")
        super().on_activate(state) # This completes the transition
        self.get_logger().info("Node is active, subscriptions are now receiving messages.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_deactivate() is called.")
        super().on_deactivate(state) # This completes the transition
        self.get_logger().info("Node is inactive, subscriptions have stopped.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_cleanup() is called.")
        
        # Destroy all ROS entities
        self.destroy_subscription(self.odom_subscriber)
        self.destroy_subscription(self.imu_subscriber)
        self.destroy_subscription(self.stereo_rgb_subscriber)
        self.destroy_subscription(self.stereo_depth_subscriber)
        self.destroy_subscription(self.stereo_info_subscriber)
        self.destroy_subscription(self.stereo_visual_subscriber)
        self.destroy_subscription(self.battery_subscriber)

        self.tf_broadcaster = None

        self.destroy_publisher(self.odom_pub)
        self.destroy_publisher(self.imu_pub)
        self.destroy_publisher(self.battery_pub)
        self.destroy_publisher(self.stereo_rgb_pub)
        self.destroy_publisher(self.stereo_depth_pub)
        self.destroy_publisher(self.stereo_info_pub)
        self.destroy_publisher(self.stereo_visual_pub)
        
        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_shutdown() is called.")
        # Ensure cleanup is called
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


    def odom_subscriber_callback(self, msg):
        # Odometer gives x, y, theta (yaw). Convert yaw to quaternion.
        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)

        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = f'{self.prefix}odom'
        transform.child_frame_id = f'{self.prefix}base_link'
        transform.transform.translation.x = float(msg.x)
        transform.transform.translation.y = float(msg.y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = f'{self.prefix}odom'
        odom.child_frame_id = f'{self.prefix}base_link'
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)




    def imu_subscriber_callback(self, msg):
        stamp = self.get_clock().now().to_msg()

        # --- IMU ---
        imu_state = msg.imu_state
        roll  = float(imu_state.rpy[0])
        pitch = float(imu_state.rpy[1])
        yaw   = float(imu_state.rpy[2])

        # Convert RPY to quaternion
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = f'{self.prefix}imu'
        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        imu.angular_velocity.x = float(imu_state.gyro[0])
        imu.angular_velocity.y = float(imu_state.gyro[1])
        imu.angular_velocity.z = float(imu_state.gyro[2])
        imu.linear_acceleration.x = float(imu_state.acc[0])
        imu.linear_acceleration.y = float(imu_state.acc[1])
        imu.linear_acceleration.z = float(imu_state.acc[2])
        self.imu_pub.publish(imu)

    def stereo_rgb_callback(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_rgb_time < self._rgb_min_interval:
            return  # drop frame to reduce CPU load
        self._last_rgb_time = now

        if msg.encoding.lower() == 'nv12':
            # NV12 is YUV 4:2:0 semi-planar; height of the NV12 buffer is h*3//2
            h = msg.height
            w = msg.width
            yuv = self._np.frombuffer(msg.data, dtype=self._np.uint8).reshape((h * 3 // 2, w))
            rgb = self._cv2.cvtColor(yuv, self._cv2.COLOR_YUV2RGB_NV12)
            out = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            out.header = msg.header
            self.stereo_rgb_pub.publish(out)
        else:
            self.stereo_rgb_pub.publish(msg)

    def stereo_depth_callback(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_depth_time < self._depth_min_interval:
            return
        self._last_depth_time = now
        if msg.encoding == 'mono16':
            msg.encoding = '16UC1'
        self.stereo_depth_pub.publish(msg)

    def stereo_info_callback(self, msg: CameraInfo):
        self.stereo_info_pub.publish(msg)

    def stereo_visual_callback(self, msg: Image):
        self.stereo_visual_pub.publish(msg)

    def battery_callback(self, msg: BoosterBatteryState):
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = f'{self.prefix}base'
        battery.voltage = msg.voltage
        battery.current = msg.current
        battery.percentage = msg.soc / 100.0
        self.battery_pub.publish(battery)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRead()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()