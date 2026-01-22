#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist


class LifecycleZeroVelocityPublisher(LifecycleNode):

    def __init__(self):
        super().__init__('zero_vel_publisher')
        self.zero_vel_pub = None
        self.timer = None

    def on_configure(self, state):
        self.get_logger().info('In on_configure(), creating publisher.')
        self.zero_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel_zero',
            1
        )
        self.get_logger().info('Publisher for cmd_vel_zero created.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('In on_activate(), starting timer.')
        self.timer = self.create_timer(0.1, self.publish_zero_velocity)
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('In on_deactivate(), stopping timer.')
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('In on_cleanup(), destroying publisher and timer.')
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.zero_vel_pub is not None:
            self.destroy_publisher(self.zero_vel_pub)
            self.zero_vel_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('In on_shutdown(), cleaning up resources.')
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.zero_vel_pub is not None:
            self.destroy_publisher(self.zero_vel_pub)
            self.zero_vel_pub = None
        return TransitionCallbackReturn.SUCCESS

    def publish_zero_velocity(self):
        zero_twist = Twist()
        if self.zero_vel_pub is not None:
            self.zero_vel_pub.publish(zero_twist)

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleZeroVelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()