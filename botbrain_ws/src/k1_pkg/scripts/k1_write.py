#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from booster_interface.srv import RpcService
from booster_interface.msg import BoosterApiReqMsg
from bot_custom_interfaces.srv import Mode
from geometry_msgs.msg import Twist
import json
import threading
import time


class RobotWriteNode(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_robot_write_node')

        self.callback_group = None
        self.cmd_vel_subscription = None
        self.rpc_client = None
        self.set_mode_srv = None

        # Maps mode name -> RobotMode int (from robot_shared.hpp)
        self._mode_map = {
            'damping': 0,   # kDamping
            'prepare': 1,   # kPrepare (stand)
            'walking': 2,   # kWalking (humanlike gait)
            'soccer': 4,    # kSoccer (soccer gait)
        }

        self.get_logger().info("RobotWriteNode created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_subscription_callback, 1,
            callback_group=self.callback_group)

        self.rpc_client = self.create_client(
            RpcService, 'booster_rpc_service', callback_group=self.callback_group)

        self.set_mode_srv = self.create_service(
            Mode, 'change_mode', self.handle_change_mode, callback_group=self.callback_group)

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)
        self.get_logger().info("Node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        super().on_deactivate(state)
        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")
        self.destroy_subscription(self.cmd_vel_subscription)
        self.destroy_client(self.rpc_client)
        self.destroy_service(self.set_mode_srv)
        self.get_logger().info("Node resources cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    def cmd_vel_subscription_callback(self, msg):
        lx = msg.linear.x
        ly = msg.linear.y
        az = msg.angular.z

        req_msg = BoosterApiReqMsg()
        req_msg.api_id = 2001  # kMove
        req_msg.body = json.dumps({"vx": lx, "vy": ly, "vyaw": az})

        request = RpcService.Request()
        request.msg = req_msg
        future = self.rpc_client.call_async(request)
        future.add_done_callback(self._move_response_callback)

    def _move_response_callback(self, future):
        try:
            result = future.result()
            if result is None:
                self.get_logger().warn('Move RPC call returned None')
        except Exception as e:
            self.get_logger().error(f'Move RPC call raised exception: {e}')

    def handle_change_mode(self, request, response):
        mode_int = self._mode_map.get(request.mode.lower())
        if mode_int is None:
            response.success = False
            response.message = f"Unknown mode '{request.mode}'. Valid: {list(self._mode_map.keys())}"
            return response

        req_msg = BoosterApiReqMsg()
        req_msg.api_id = 2000  # kChangeMode
        req_msg.body = json.dumps({"mode": mode_int})

        rpc_request = RpcService.Request()
        rpc_request.msg = req_msg
        future = self.rpc_client.call_async(rpc_request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait(timeout=2.0)

        if future.done() and future.result() is not None:
            response.success = True
            response.message = f"Mode changed to '{request.mode}'"
        else:
            response.success = False
            response.message = "RPC call timed out or failed"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotWriteNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.trigger_configure()
    node.trigger_activate()


    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
