#!/usr/bin/env python3
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from booster_interface.msg import Odometer, LowState, ImuState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, BatteryState, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class RobotRead(LifecycleNode):

    def __init__(self):
        super().__init__('robot_read_node')

        # Parameters must be declared in the constructor.
        self.declare_parameter('prefix', '')
        self.prefix = '' 

        # Initialize all ROS communicators to None.
        self.sport_mode_subscriber = None
        self.low_state_state_subscriber = None
        self.imu_subscriber = None
        self.pose_subscriber = None
        self.lidar_subcriber = None
        
        self.tf_broadcaster = None

        self.odom_pub = None
        self.imu_pub = None
        self.lidar_pub = None
        self.battery_pub = None
        self.imu_temp_pub = None
        self.joint_state_pub = None

        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        # Get the parameter value now that the node is being configured.
        self.prefix = self.get_parameter('prefix').value
        self.get_logger().info(f"Using prefix: '{self.prefix}'")
        
        # Create subscribers
        self.odom_subscriber = self.create_subscription(Odometer, '/odometer_state', self.odom_subscriber_callback, 10)
        self.imu_subscriber = self.create_subscription(LowState, '/low_state', self.imu_subscriber_callback, 10)

        # self.low_state_state_subscriber = self.create_subscription(LowState, '/lf/lowstate', self.low_state_subscriber_callback, 10)
        # self.pose_subscriber = self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.publish_pose_stamped, 10)
        # self.lidar_subcriber = self.create_subscription(PointCloud2, '/utlidar/cloud', self.publish_lidar, 10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.imu_temp_pub = self.create_publisher(Float32, 'imu_temp', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

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
        self.destroy_subscription(self.low_state_state_subscriber)
        self.destroy_subscription(self.pose_subscriber)
        self.destroy_subscription(self.lidar_subcriber)

        self.tf_broadcaster = None

        self.destroy_publisher(self.odom_pub)
        self.destroy_publisher(self.imu_pub)
        self.destroy_publisher(self.lidar_pub)
        self.destroy_publisher(self.battery_pub)
        self.destroy_publisher(self.imu_temp_pub)
        self.destroy_publisher(self.joint_state_pub)
        
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

    def publish_pose_stamped(self, msg):

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = f'{self.prefix}odom'
        transform.child_frame_id = f'{self.prefix}base_link'
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = f'{self.prefix}odom'
        odom.child_frame_id = f'{self.prefix}base_link'
        odom.pose.pose = msg.pose
        self.odom_pub.publish(odom)

    def sport_subscriber_callback(self, msg):

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg() 
        imu.header.frame_id = f'{self.prefix}imu'

        imu.orientation.x = np.float64(msg.imu_state.quaternion[0])
        imu.orientation.y = np.float64(msg.imu_state.quaternion[1])
        imu.orientation.z = np.float64(msg.imu_state.quaternion[2])
        imu.orientation.w = np.float64(msg.imu_state.quaternion[3])
        imu.angular_velocity.x = np.float64(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = np.float64(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = np.float64(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = np.float64(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = np.float64(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = np.float64(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

        imu_temp = Float32()
        imu_temp.data = np.float64(msg.imu_state.temperature)
        self.imu_temp_pub.publish(imu_temp)

    def publish_lidar(self, msg):

        lidar = msg
        lidar.header.stamp = self.get_clock().now().to_msg() 
        lidar.header.frame_id = f'{self.prefix}radar'
        self.lidar_pub.publish(lidar)

    def low_state_subscriber_callback(self, msg):

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = f'{self.prefix}base'
        battery.voltage = msg.power_v
        battery.current = float(msg.bms_state.current)
        battery.percentage = float(msg.bms_state.soc) / 100.0
        self.battery_pub.publish(battery)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'{self.prefix}FL_hip_joint', 
                                f'{self.prefix}FL_thigh_joint', 
                                f'{self.prefix}FL_calf_joint',
                               f'{self.prefix}FR_hip_joint', 
                               f'{self.prefix}FR_thigh_joint', 
                               f'{self.prefix}FR_calf_joint',
                               f'{self.prefix}RL_hip_joint', 
                               f'{self.prefix}RL_thigh_joint', 
                               f'{self.prefix}RL_calf_joint',
                               f'{self.prefix}RR_hip_joint', 
                               f'{self.prefix}RR_thigh_joint', 
                               f'{self.prefix}RR_calf_joint']
        joint_state_msg.position = [
            np.float64(msg.motor_state[3].q), 
            np.float64(msg.motor_state[4].q), 
            np.float64(msg.motor_state[5].q),
            np.float64(msg.motor_state[0].q), 
            np.float64(msg.motor_state[1].q), 
            np.float64(msg.motor_state[2].q),
            np.float64(msg.motor_state[9].q), 
            np.float64(msg.motor_state[10].q), 
            np.float64(msg.motor_state[11].q),
            np.float64(msg.motor_state[6].q), 
            np.float64(msg.motor_state[7].q), 
            np.float64(msg.motor_state[8].q),
        ]
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRead()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()