#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState

class RobotReadNode(LifecycleNode):

    def __init__(self):
        super().__init__('robot_read_node')

        # Parameters must be declared in the constructor.
        self.declare_parameter('prefix', '')
        self.prefix = '' 

        # Initialize all ROS communicators to None.
        self.battery_state_subscriber = None
        self.odometry_subscriber = None
        

        self.odom_pub = None
        self.imu_pub = None
        self.battery_pub = None

        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        # Get the parameter value now that the node is being configured.
        self.prefix = self.get_parameter('prefix').value
        self.get_logger().info(f"Using prefix: '{self.prefix}'")
        
        # Create subscribers
        self.odometry_subscriber = self.create_subscription(Odometry, '/rt/odometry_state', self.odometry_subscriber_callback, 10)
        self.battery_state_subscriber = self.create_subscription(BatteryState, '/battery_state', self.battery_state_subscriber_callback, 10)

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)

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
        self.destroy_subscription(self.odometry_subscriber)
        self.destroy_subscription(self.battery_state_subscriber)

        self.destroy_publisher(self.odom_pub)
        self.destroy_publisher(self.imu_pub)
        self.destroy_publisher(self.battery_pub)
        
        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_shutdown() is called.")
        # Ensure cleanup is called
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


    def odometry_subscriber_callback(self, msg):

        odom_msg = Odometry()
        odom_msg = msg
        self.odom_pub.publish(odom_msg)

    def battery_state_subscriber_callback(self, msg):

        battery_msg = BatteryState()
        battery_msg = msg
        self.battery_pub.publish(battery_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotReadNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()