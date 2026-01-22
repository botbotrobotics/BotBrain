#!/usr/bin/env python3
import rclpy

# Key imports for a lifecycle node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node # Still needed for type hints in older rclpy versions
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from unitree_api.msg import Request
from joystick_bot.msg import ControllerButtonsState
from bot_custom_interfaces.srv import Mode, Pose, SwitchGait,CurrentMode
import json
import time
import numpy as np

class OperationModes(LifecycleNode):

    def __init__(self):
        # Initialize the lifecycle node in the 'unconfigured' state
        super().__init__('controller_commands_node')
        
        # ROS entities that will be created in on_configure()/on_activate()
        self.button_state_subscription = None
        self.timer = None
        
        # Service Clients
        self.set_mode_srv_cli = None        
        self.get_mode_srv_cli = None
        
        # Internal state variables
        self.control_dict = {}
        self.button_states = {}

        # Callback group
        self.cb_group = None

        # Long-press botton detection
        self._l2b_hold_started_at = None
        self._l2b_required_secs = 5.0
        
        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() called: configuring node.")
        
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_activate() called: activating node.")

        # Subscription for joystick/controller button states
        self.button_state_subscription = self.create_subscription(
            ControllerButtonsState,
            'button_state',
            self.button_subscription_callback,
            1,
            callback_group=self.cb_group)
        
        # Create service clients for mode control and mode query
        self.set_mode_srv_cli = self.create_client(Mode, 'mode', callback_group=self.cb_group)
        self.get_mode_srv_cli = self.create_client(CurrentMode, 'current_mode', callback_group=self.cb_group)

        if not self.set_mode_srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service 'mode' unavailable")
            return TransitionCallbackReturn.FAILED
        if not self.get_mode_srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service 'current_mode' unavailable")
            return TransitionCallbackReturn.FAILED

        # Periodic timer to process button states and send mode commands
        self.timer = self.create_timer(0.2, self.timer_callback, callback_group=self.cb_group)

        self.get_logger().info("Node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_deactivate() called: deactivating node.")

        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        if self.button_state_subscription:
            self.destroy_subscription(self.button_state_subscription)
            self.button_state_subscription = None

        if self.set_mode_srv_cli:
            self.destroy_client(self.set_mode_srv_cli)
            self.set_mode_srv_cli = None

        if self.get_mode_srv_cli:
            self.destroy_client(self.get_mode_srv_cli)
            self.get_mode_srv_cli = None
        
        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() called: cleaning up node.")
        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() called: shutting down node.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    def button_subscription_callback(self, msg: ControllerButtonsState):
        # Store the latest button states from the controller.
        self.button_states = {
            'start': msg.start_button, 
            'select': msg.select_button,
            'L2': msg.l2_button, 
            'L1': msg.l1_button, 
            'R1': msg.r1_button, 
            'R2': msg.r2_button,
            'A': msg.a_button, 
            'B': msg.b_button, 
            'Y': msg.y_button, 
            'X': msg.x_button,
            'right': msg.right_button, 
            'left': msg.left_button, 
            'up': msg.up_button, 
            'down': msg.down_button
        }
    
    def timer_callback(self):

        # Get robot mode
        self.mode = self.get_current_mode()
        if not self.mode:
            return

        # Long-press treatment
        if self.button_states.get('L2') and self.button_states.get('B'):
            if self._l2b_hold_started_at is None:
                self._l2b_hold_started_at = time.monotonic()

            held = time.monotonic() - self._l2b_hold_started_at
            if self.mode == "run" and held >= self._l2b_required_secs:
                req = Mode.Request(); 
                req.mode = 'damp'
                self.set_mode_srv_cli.call_async(req)
                self._l2b_hold_started_at = None  
                
        else:
            # Reset hold timer when combination is released
            self._l2b_hold_started_at = None

        
        if self.control_dict != self.button_states:
            self.get_logger().info(
                f"Button state changed.\nOld: {self.control_dict}\nNew: {self.button_states}"
            )

            # 'start' button: if in 'run' -> send 'start' mode request
            if self.button_states.get('start'):
                if self.mode == "run":
                    requ = Mode.Request()
                    requ.mode = 'start'
                    self.set_mode_srv_cli.call_async(requ)
                self.get_logger().info('start')
            
            # L2 + B: go to 'damp' from zero_torque/squat/preparation
            elif self.button_states.get('L2') and self.button_states.get('B'):
                if self.mode == "zero_torque" or self.mode == "squat" or self.mode == "preparation":
                    requ = Mode.Request()
                    requ.mode = 'damp'
                    self.set_mode_srv_cli.call_async(requ)
                    self.get_logger().info('L2+B')

            # L2 + up: 'damp' -> 'preparation'
            elif self.button_states.get('L2') and self.button_states.get('up'):
                if self.mode == "damp":
                    requ = Mode.Request()
                    requ.mode = 'preparation'
                    self.set_mode_srv_cli.call_async(requ)
                    self.get_logger().info('L2+up')

            # R2 + A: 'preparation' -> 'run'
            elif self.button_states.get('R2') and self.button_states.get('A'):
                if self.mode == "preparation":
                    requ = Mode.Request()
                    requ.mode = 'run'
                    self.set_mode_srv_cli.call_async(requ)
                    self.get_logger().info('R2+A')
            
            # L2 + A: from 'run' or 'damp' to 'squat'
            elif self.button_states['L2'] and self.button_states['A']:
                if self.mode == "run" or self.mode == "damp":
                    requ = Mode.Request()
                    requ.mode = 'squat'
                    self.set_mode_srv_cli.call_async(requ)
                self.get_logger().info('L2+A')

            # After processing commands, update the stored state
            self.control_dict = self.button_states.copy()

    # Utils
    def get_current_mode(self, timeout=0.5):
        if self.get_mode_srv_cli is None:
            return None
        req = CurrentMode.Request()
        future = self.get_mode_srv_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done() and not future.cancelled():
            try:
                resp = future.result()
                return resp.mode
            except Exception as e:
                self.get_logger().error(f"Erro in current_mode: {e}")
                return None
        else:
            self.get_logger().warn("Timeout in current_mode()")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = OperationModes()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()