#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    RobotMode,
)


class RobotWriteSdk(LifecycleNode):

    def __init__(self):
        super().__init__('robot_write_sdk_node')

        self.declare_parameter('network_interface', '')

        self._client = None
        self._cmd_vel_sub = None
        self._callback_group = None

        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        network_interface = self.get_parameter('network_interface').value

        if network_interface:
            self.get_logger().info(f"Initializing ChannelFactory with interface: '{network_interface}'")
            ChannelFactory.Instance().Init(0, network_interface)
        else:
            self.get_logger().info("Initializing ChannelFactory with domain 0 (local)")
            ChannelFactory.Instance().Init(0)

        self._client = B1LocoClient()
        self._client.Init()

        self._callback_group = ReentrantCallbackGroup()
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)

        res = self._client.ChangeMode(RobotMode.kWalking)
        if res != 0:
            self.get_logger().error(f"Failed to switch robot to walking mode: error={res}")
            super().on_deactivate(state)
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Robot is in walking mode. Ready to receive cmd_vel.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")

        # Stop movement before deactivating
        self._client.Move(0.0, 0.0, 0.0)

        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")

        if self._cmd_vel_sub is not None:
            self.destroy_subscription(self._cmd_vel_sub)
            self._cmd_vel_sub = None

        self._client = None
        self._callback_group = None

        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        if self._client is not None:
            self._client.Move(0.0, 0.0, 0.0)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # cmd_vel callback
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist):
        """Forward /cmd_vel to the robot via the Booster SDK loco client.

        Mapping:
          linear.x  -> forward / backward velocity
          linear.y  -> lateral (strafe) velocity
          angular.z -> yaw rotation rate
        """
        x = float(msg.linear.x)
        y = float(msg.linear.y)
        z = float(msg.angular.z)

        res = self._client.Move(x, y, z)
        if res != 0:
            self.get_logger().warn(f"Move() returned error={res} for cmd_vel ({x}, {y}, {z})")


def main(args=None):
    rclpy.init(args=args)
    node = RobotWriteSdk()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
