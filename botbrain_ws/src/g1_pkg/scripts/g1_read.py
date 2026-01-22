#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32
from unitree_hg.msg import BmsState, LowState
from unitree_go.msg import SportModeState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

"""
    Lifecycle node responsible for reading low-level robot states (BMS, low state,
    sport mode state) and republishing them as standard ROS messages:
    - BatteryState
    - JointState
    - Odometry
    - Imu
    - TF odom -> base_link
"""
class RobotRead(LifecycleNode):

    def __init__(self):
        # Initialize lifecycle node in the 'unconfigured' state
        super().__init__('robot_read_node')

        self.declare_parameter('prefix', '')
        self.prefix = '' 

        # Initialize all ROS communicators to None.
        self.low_bms_state_subscriber = None
        self.low_state_subscriber = None
        self.sport_mode_state_subscriber = None

        self.battery_pub = None
        self.joint_state_pub = None
        self.odom_pub = None
        self.imu_pub = None
        self.imu_temp_pub = None

        self.tf_broadcaster = None

        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() called: configuring node.")

        # Get the parameter 
        self.prefix = self.get_parameter('prefix').value
        self.get_logger().info(f"Using frame/joint prefix: '{self.prefix}'")

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_activate() called: activating node.")

        # Create publishers
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.imu_temp_pub = self.create_publisher(Float32, 'imu_temp', 10)

        # Create subscribers
        self.low_bms_state_subscriber = self.create_subscription(BmsState, '/lf/bmsstate', self.low_bms_state_callback, 10)
        self.low_state_subscriber = self.create_subscription(LowState, '/lf/lowstate', self.low_state_callback, 10)
        self.sport_mode_state_subscriber = self.create_subscription(SportModeState, '/lf/odommodestate', self.sport_mode_state_callback, 10)

        # TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Node is activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_deactivate() called: deactivating node.")

        if self.battery_pub:
            self.destroy_publisher(self.battery_pub)
            self.battery_pub = None

        if self.joint_state_pub:
            self.destroy_publisher(self.joint_state_pub)
            self.joint_state_pub = None

        if self.odom_pub:
            self.destroy_publisher(self.odom_pub)
            self.odom_pub = None

        if self.imu_pub:
            self.destroy_publisher(self.imu_pub)
            self.imu_pub = None
            
        if self.imu_temp_pub:
            self.destroy_publisher(self.imu_temp_pub)
            self.imu_temp_pub = None

        if self.low_bms_state_subscriber:
            self.destroy_subscription(self.low_bms_state_subscriber)
            self.low_bms_state_subscriber = None

        if self.low_state_subscriber:
            self.destroy_subscription(self.low_state_subscriber)
            self.low_state_subscriber = None

        if self.sport_mode_state_subscriber:
            self.destroy_subscription(self.sport_mode_state_subscriber)
            self.sport_mode_state_subscriber = None

        if self.tf_broadcaster:
            self.tf_broadcaster = None

        self.get_logger().info("Node deactivated successfully.")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_cleanup() called: cleaning up node resources.")

        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_shutdown() called: shutting down node.")

        # Ensure cleanup is called
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS
 
    # Convert low-level BMS state into a standard BatteryState message.
    def low_bms_state_callback(self, msg):
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = f'{self.prefix}base'
        battery.voltage = float(msg.bmsvoltage[0]) / 1000.0
        battery.current = float(msg.current)  / 1000.0
        battery.percentage = float(msg.soc) / 100.0
        self.battery_pub.publish(battery)

    # Convert low-level motor states into a JointState message.
    def low_state_callback(self, msg):
        joint_state_msg = self.create_joint_msg(msg)
        self.joint_state_pub.publish(joint_state_msg)

    """
        Convert sport mode state into:
        - IMU message
        - IMU temperature
        - Odometry message
        - TF transform from odom to base_link
    """
    def sport_mode_state_callback(self, msg):
        stamp = self.get_clock().now().to_msg()

        # Publish IMU
        imu_msg = self.create_imu_msg(msg, stamp)
        self.imu_pub.publish(imu_msg)

        # Publish IMU temperature
        imu_temp = Float32()
        imu_temp.data = np.float64(msg.imu_state.temperature)
        self.imu_temp_pub.publish(imu_temp)
        
        # Publish odometry
        odom_msg = self.create_odom_msg(msg, stamp)
        self.odom_pub.publish(odom_msg)

        # Publish TF transform odom -> base_link
        tf_msg = self.create_odom_tf(msg, stamp)
        self.tf_broadcaster.sendTransform(tf_msg)

    # Build a sensor_msgs/Imu message from SportModeState.
    def create_imu_msg(self, msg, stamp):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = f'{self.prefix}imu'

        # Orientation (quaternion)
        imu.orientation.w = np.float64(msg.imu_state.quaternion[0])
        imu.orientation.x = np.float64(msg.imu_state.quaternion[1])
        imu.orientation.y = np.float64(msg.imu_state.quaternion[2])
        imu.orientation.z = np.float64(msg.imu_state.quaternion[3])

        # Angular velocity (gyro)
        imu.angular_velocity.x = np.float64(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = np.float64(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = np.float64(msg.imu_state.gyroscope[2])

        # Linear acceleration
        imu.linear_acceleration.x = np.float64(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = np.float64(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = np.float64(msg.imu_state.accelerometer[2])

        return imu

    # Build a nav_msgs/Odometry message from SportModeState.
    def create_odom_msg(self, msg, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = f'{self.prefix}odom'
        odom.child_frame_id = f'{self.prefix}base_link'

        # Position
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2]) 

        # Orientation (copy from IMU quaternion)
        odom.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])
        odom.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])

        # Linear velocities
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])

        # Angular velocity around Z (yaw rate)
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(msg.yaw_speed)

        return odom

    # Build a TransformStamped for the odom -> base_link transform.
    def create_odom_tf(self, msg, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = f'{self.prefix}odom'
        transform.child_frame_id = f'{self.prefix}base_link'

        # Translation
        transform.transform.translation.x = float(msg.position[0])
        transform.transform.translation.y = float(msg.position[1])
        transform.transform.translation.z = float(msg.position[2])

        # Rotation (copy quaternion from IMU)
        transform.transform.rotation.w = float(msg.imu_state.quaternion[0])
        transform.transform.rotation.x = float(msg.imu_state.quaternion[1])
        transform.transform.rotation.y = float(msg.imu_state.quaternion[2])
        transform.transform.rotation.z = float(msg.imu_state.quaternion[3])

        return transform

    # Build a JointState message for all relevant joints from LowState.motor_state.
    def create_joint_msg(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Joint names follow the naming convention used in the URDF
        joint_state_msg.name = [
            f'{self.prefix}left_hip_pitch_joint',
            f'{self.prefix}left_hip_roll_joint',
            f'{self.prefix}left_hip_yaw_joint',
            f'{self.prefix}left_knee_joint',
            f'{self.prefix}left_ankle_pitch_joint',
            f'{self.prefix}left_ankle_roll_joint',
            f'{self.prefix}right_hip_pitch_joint',
            f'{self.prefix}right_hip_roll_joint',
            f'{self.prefix}right_hip_yaw_joint',
            f'{self.prefix}right_knee_joint',
            f'{self.prefix}right_ankle_pitch_joint',
            f'{self.prefix}right_ankle_roll_joint',
            f'{self.prefix}waist_yaw_joint',
            f'{self.prefix}left_shoulder_pitch_joint',
            f'{self.prefix}left_shoulder_roll_joint',
            f'{self.prefix}left_shoulder_yaw_joint',
            f'{self.prefix}left_elbow_joint',
            f'{self.prefix}left_wrist_roll_joint',
            f'{self.prefix}right_shoulder_pitch_joint',
            f'{self.prefix}right_shoulder_roll_joint',
            f'{self.prefix}right_shoulder_yaw_joint',
            f'{self.prefix}right_elbow_joint',
            f'{self.prefix}right_wrist_roll_joint',
        ]

        # Map motor_state indices to joint positions.
        joint_state_msg.position = [
            np.float64(msg.motor_state[0].q),   # left_hip_pitch
            np.float64(msg.motor_state[1].q),   # left_hip_roll
            np.float64(msg.motor_state[2].q),   # left_hip_yaw
            np.float64(msg.motor_state[3].q),   # left_knee
            np.float64(msg.motor_state[4].q),   # left_ankle_pitch
            np.float64(msg.motor_state[5].q),   # left_ankle_roll
            np.float64(msg.motor_state[6].q),   # right_hip_pitch
            np.float64(msg.motor_state[7].q),   # right_hip_roll
            np.float64(msg.motor_state[8].q),   # right_hip_yaw
            np.float64(msg.motor_state[9].q),   # right_knee
            np.float64(msg.motor_state[10].q),  # right_ankle_pitch
            np.float64(msg.motor_state[11].q),  # right_ankle_roll
            np.float64(msg.motor_state[12].q),  # waist_yaw
            np.float64(msg.motor_state[15].q),  # left_shoulder_pitch
            np.float64(msg.motor_state[16].q),  # left_shoulder_roll
            np.float64(msg.motor_state[17].q),  # left_shoulder_yaw
            np.float64(msg.motor_state[18].q),  # left_elbow
            np.float64(msg.motor_state[19].q),  # left_wrist_roll
            np.float64(msg.motor_state[22].q),  # right_shoulder_pitch
            np.float64(msg.motor_state[23].q),  # right_shoulder_roll
            np.float64(msg.motor_state[24].q),  # right_shoulder_yaw
            np.float64(msg.motor_state[25].q),  # right_elbow
            np.float64(msg.motor_state[26].q),  # right_wrist_roll
        ]
        return joint_state_msg

def main(args=None):
    rclpy.init(args=args)
    node = RobotRead()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()