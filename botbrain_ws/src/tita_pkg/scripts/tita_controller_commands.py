#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from joystick_bot.msg import ControllerButtonsState
from std_msgs.msg import String
from bot_custom_interfaces.srv import Mode

class ControllerCommands(LifecycleNode):
    def __init__(self):
        super().__init__('controller_commands_node')
        self.tita_namespace = ''
        self.button_state_subscription = None
        self.fsm_mode_subscription = None
        self.mode_srv_cli = None
        self.timer = None

        self.control_dict = {}
        self.button_states = {}
        self.op_mode = None

    def on_configure(self, state):
        self.get_logger().info('on_configure() is called.')
        self.declare_parameter('tita_namespace', '')
        self.tita_namespace = self.get_parameter('tita_namespace').value

        # Build namespace prefix with proper slash handling
        namespace_prefix = f'/{self.tita_namespace}' if self.tita_namespace else ''

        self.button_state_subscription = self.create_subscription(ControllerButtonsState, 'button_state', self.button_subscription_callback, 1)
        self.fsm_mode_subscription = self.create_subscription(String, f'{namespace_prefix}/locomotion/body/fsm_mode', self.fsm_mode_callback, 1)
        self.mode_srv_cli = self.create_client(Mode, 'mode')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Node configured. Waiting for manual activation...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate() is called.')
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate() is called.')
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup() is called.')
        self.destroy_subscription(self.button_state_subscription)
        self.destroy_subscription(self.fsm_mode_subscription)
        self.destroy_client(self.mode_srv_cli)
        self.destroy_timer(self.timer)
        self.get_logger().info('Node cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    # --- Callbacks ---
    
    def button_subscription_callback(self, msg):
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
    
    def fsm_mode_callback(self, msg):
        self.op_mode = msg.data

    def timer_callback(self):
        if self.control_dict != self.button_states:
            if self.button_states.get('L2') and self.button_states.get('A'):
                if self.op_mode in ['transform_down', 'idle']:
                    self.get_logger().info("Calling 'transform_up' service")
                    req = Mode.Request()
                    req.mode = 'stand_up'
                    if self.mode_srv_cli.wait_for_service(timeout_sec=1.0):
                        self.mode_srv_cli.call_async(req)
                else:
                    self.get_logger().info("Calling 'transform_down' service")
                    req = Mode.Request()
                    req.mode = 'stand_down'
                    if self.mode_srv_cli.wait_for_service(timeout_sec=1.0):
                        self.mode_srv_cli.call_async(req)

            self.control_dict = self.button_states.copy()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerCommands()
    executor = MultiThreadedExecutor()
    executor.add_node(node) 
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()