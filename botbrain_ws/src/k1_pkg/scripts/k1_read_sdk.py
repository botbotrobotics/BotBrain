#!/usr/bin/env python3
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LowStateSubscriber,
    B1OdometerStateSubscriber,
)


class RobotReadSdk(LifecycleNode):

    def __init__(self):
        super().__init__('robot_read_sdk_node')

        self.declare_parameter('prefix', '')
        self.declare_parameter('network_interface', '')
        self.prefix = ''

        self.tf_broadcaster = None
        self.odom_pub = None
        self.imu_pub = None
        self.joint_state_pub = None

        self._low_state_sub = None
        self._odometer_sub = None

        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.prefix = self.get_parameter('prefix').value
        network_interface = self.get_parameter('network_interface').value

        self.get_logger().info(f"Using prefix: '{self.prefix}'")

        # Initialize Booster SDK channel factory
        if network_interface:
            self.get_logger().info(f"Initializing ChannelFactory with interface: '{network_interface}'")
            ChannelFactory.Instance().Init(0, network_interface)
        else:
            self.get_logger().info("Initializing ChannelFactory with domain 0 (local)")
            ChannelFactory.Instance().Init(0)

        # Create TF broadcaster and ROS publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscribe via Booster SDK (DDS under the hood)
        self._low_state_sub = B1LowStateSubscriber(self._low_state_callback)
        self._low_state_sub.InitChannel()

        self._odometer_sub = B1OdometerStateSubscriber(self._odometer_callback)
        self._odometer_sub.InitChannel()

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)
        self.get_logger().info("Node is active, SDK subscribers are receiving data.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")
        self._close_sdk_channels()
        self.tf_broadcaster = None
        self.destroy_publisher(self.odom_pub)
        self.destroy_publisher(self.imu_pub)
        self.destroy_publisher(self.joint_state_pub)
        self.odom_pub = None
        self.imu_pub = None
        self.joint_state_pub = None
        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        self._close_sdk_channels()
        return TransitionCallbackReturn.SUCCESS

    def _close_sdk_channels(self):
        if self._low_state_sub is not None:
            try:
                self._low_state_sub.CloseChannel()
            except Exception:
                pass
            self._low_state_sub = None
        if self._odometer_sub is not None:
            try:
                self._odometer_sub.CloseChannel()
            except Exception:
                pass
            self._odometer_sub = None

    # ------------------------------------------------------------------
    # SDK callbacks  (called from DDS threads — publishers are thread-safe)
    # ------------------------------------------------------------------

    def _odometer_callback(self, msg):
        """Receives odometer data from the Booster SDK and publishes Odometry + TF."""
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

    def _low_state_callback(self, msg):
        """Receives low-level state from the Booster SDK and publishes IMU + JointState."""
        stamp = self.get_clock().now().to_msg()

        # --- IMU ---
        imu_state = msg.imu_state
        roll  = float(imu_state.rpy[0])
        pitch = float(imu_state.rpy[1])
        yaw   = float(imu_state.rpy[2])

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

        # --- Joint States ---
        motors = msg.motor_state_serial
        if len(motors) >= 12:
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = [
                f'{self.prefix}FR_hip_joint',   f'{self.prefix}FR_thigh_joint', f'{self.prefix}FR_calf_joint',
                f'{self.prefix}FL_hip_joint',   f'{self.prefix}FL_thigh_joint', f'{self.prefix}FL_calf_joint',
                f'{self.prefix}RR_hip_joint',   f'{self.prefix}RR_thigh_joint', f'{self.prefix}RR_calf_joint',
                f'{self.prefix}RL_hip_joint',   f'{self.prefix}RL_thigh_joint', f'{self.prefix}RL_calf_joint',
            ]
            joint_state.position = [float(motors[i].q)  for i in range(12)]
            joint_state.velocity = [float(motors[i].dq) for i in range(12)]
            self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = RobotReadSdk()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
