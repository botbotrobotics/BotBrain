#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from booster_interface.srv import RpcService
from booster_interface.msg import BoosterApiReqMsg
import json


class RobotWriteNode(LifecycleNode):

    def __init__(self):
        super().__init__('robot_write_node')

        self.callback_group = None
        self.cmd_vel_subscription = None
        self.rpc_client = None

        self.get_logger().info("K1 Write Node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.callback_group = ReentrantCallbackGroup()

        # RPC client to send commands to the K1 robot
        self.rpc_client = self.create_client(
            RpcService, 'booster_rpc_service', callback_group=self.callback_group)

        # Subscribe to cmd_vel_out from the navigation/control stack
        self.cmd_vel_subscription = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_callback, 1,
            callback_group=self.callback_group)

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)

        if not self.rpc_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("booster_rpc_service not available.")
            super().on_deactivate(state)
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Node activated. Forwarding cmd_vel_out to K1.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")

        # Send zero velocity to stop the robot
        self.send_move(0.0, 0.0, 0.0)

        super().on_deactivate(state)
        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")

        self.destroy_subscription(self.cmd_vel_subscription)
        self.destroy_client(self.rpc_client)

        self.get_logger().info("Node resources cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vyaw = msg.angular.z

        if vx or vy or vyaw:
            self.send_move(vx, vy, vyaw)

    def send_move(self, vx: float, vy: float, vyaw: float):
        req_msg = BoosterApiReqMsg()
        req_msg.api_id = 2001  # kMove
        req_msg.body = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})

        request = RpcService.Request()
        request.msg = req_msg

        future = self.rpc_client.call_async(request)
        future.add_done_callback(self.move_response_callback)

    def move_response_callback(self, future):
        try:
            result = future.result()
            if result.msg.status != 0:
                self.get_logger().warn(
                    f"Move RPC returned status {result.msg.status}: {result.msg.body}")
        except Exception as e:
            self.get_logger().error(f"Move RPC call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotWriteNode()
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
