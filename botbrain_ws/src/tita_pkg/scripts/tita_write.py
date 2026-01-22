#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from tita_locomotion_interfaces.msg import LocomotionCmd
from bot_custom_interfaces.srv import Mode
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import threading
import time

class RobotWrite(LifecycleNode):
    def __init__(self):
        super().__init__('robot_write_node')
        self.prefix = ''
        self.tita_namespace = ''
        self.cmd_vel_subscriber = None
        self.command_pub = None
        self.mode_service = None
        self.param_set_client = None
        self.param_setting_thread = None

    def on_configure(self, state):
        self.get_logger().info('on_configure() is called.')
        self.declare_parameter('prefix', '')
        self.declare_parameter('tita_namespace', '')
        self.prefix = self.get_parameter('prefix').value
        self.tita_namespace = self.get_parameter('tita_namespace').value

        # Build namespace prefix with proper slash handling
        namespace_prefix = f'/{self.tita_namespace}' if self.tita_namespace else ''

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel_out', self.cmd_vel_callback, 1)
        self.command_pub = self.create_lifecycle_publisher(LocomotionCmd, f'{namespace_prefix}/command/user/command', 1)
        self.mode_service = self.create_service(Mode, 'mode', self.mode_callback)
        self.param_set_client = self.create_client(SetParameters, f'{namespace_prefix}/active_command_node/set_parameters')

        self.get_logger().info('Node configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate() is called.')
        super().on_activate(state)
        
        # Start parameter setting in background thread
        self.param_setting_thread = threading.Thread(target=self._set_use_sdk_parameter)
        self.param_setting_thread.daemon = True
        self.param_setting_thread.start()
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate() is called.')
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup() is called.')
        self.destroy_subscription(self.cmd_vel_subscriber)
        self.destroy_publisher(self.command_pub)
        self.destroy_service(self.mode_service)
        self.get_logger().info('Node cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def _set_use_sdk_parameter(self):
        """Set use_sdk parameter to true with retry logic"""
        max_retries = 10
        retry_delay = 10.0  # seconds
        
        for attempt in range(max_retries):
            try:
                # Wait for the service to be available
                if not self.param_set_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f'Parameter service not available, attempt {attempt + 1}/{max_retries}')
                    time.sleep(retry_delay)
                    continue
                
                # Create parameter
                param = Parameter()
                param.name = 'use_sdk'
                param.value = ParameterValue()
                param.value.type = ParameterType.PARAMETER_BOOL
                param.value.bool_value = True
                
                # Create request
                request = SetParameters.Request()
                request.parameters = [param]
                
                # Call service
                future = self.param_set_client.call_async(request)
                
                # Wait for response with timeout
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.1)
                
                if future.done():
                    response = future.result()
                    if response and len(response.results) > 0 and response.results[0].successful:
                        self.get_logger().info('Successfully set use_sdk parameter to true')
                        return
                    else:
                        reason = response.results[0].reason if response and len(response.results) > 0 else "Unknown error"
                        self.get_logger().warn(f'Failed to set parameter: {reason}, attempt {attempt + 1}/{max_retries}')
                else:
                    self.get_logger().warn(f'Service call timeout, attempt {attempt + 1}/{max_retries}')
                    
            except Exception as e:
                self.get_logger().warn(f'Exception setting parameter: {str(e)}, attempt {attempt + 1}/{max_retries}')
            
            time.sleep(retry_delay)
        
        self.get_logger().error('Failed to set use_sdk parameter after all retry attempts')
    
    def cmd_vel_callback(self, msg):
        cmd = LocomotionCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.fsm_mode = 'balance'
        cmd.twist = msg
        self.command_pub.publish(cmd)
    
    def mode_callback(self, request, response):
        self.get_logger().info("Mode service called.")
        if request.mode == "stand_up":
            request.mode = "transform_up"
        if request.mode == "stand_down":
            request.mode = "transform_down"
        try:
            cmd = LocomotionCmd()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.fsm_mode = request.mode
            self.command_pub.publish(cmd)
            self.get_logger().info(f'Robot mode changed to: {cmd.fsm_mode}')
            
            response.success = True
            response.message = f'Successfully changed mode to {cmd.fsm_mode}'
            
        except Exception as e:
            self.get_logger().error(f'Failed to change mode: {str(e)}')
            response.success = False
            response.message = f'Failed to change mode: {str(e)}'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotWrite()
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