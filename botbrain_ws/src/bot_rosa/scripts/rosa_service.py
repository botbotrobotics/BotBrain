#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

from bot_custom_interfaces.srv import LLMPrompt
from bot_rosa.agent import Agent


class RosaServiceLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('rosa_service_node')
        self._rosa = None
        self._srv = None

        self.declare_parameter('robot_model', '')
        self.declare_parameter('verbose', False)
        self.declare_parameter('streaming', False)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring RosaServiceLifecycleNode...')
        try:
            verbose = self.get_parameter('verbose').value
            streaming = self.get_parameter('streaming').value
            robot_model = self.get_parameter('robot_model').value 

            if not robot_model or not str(robot_model).strip():
                raise ValueError("Failed to load parameters")

            self._rosa = Agent(robot_model=robot_model, verbose=verbose, streaming=streaming)
            self.get_logger().info(f'Rosa resources created for robot: {robot_model}')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating RosaServiceLifecycleNode...')
        try:
            if self._srv is None:
                self._srv = self.create_service(
                    LLMPrompt,
                    'rosa_prompt',
                    self._service_callback
                )
                self.get_logger().info('Service "rosa_prompt" is now ACTIVE.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating RosaServiceLifecycleNode...')
        try:
            if self._srv is not None:
                self.destroy_service(self._srv)
                self._srv = None
                self.get_logger().info('Service "rosa_prompt" has been deactivated.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Deactivation failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up RosaServiceLifecycleNode...')
        try:

            if self._srv is not None:
                self.destroy_service(self._srv)
                self._srv = None

            self._rosa = None
            self.get_logger().info('Cleanup complete.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down RosaServiceLifecycleNode...')
        try:
            if self._srv is not None:
                self.destroy_service(self._srv)
                self._srv = None
            self._rosa = None
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Shutdown encountered an error: {e}')
            return TransitionCallbackReturn.FAILURE

  
    def _service_callback(self, request, response):
        prompt = request.input
        try:
            response.output = self._rosa.get_response(prompt)
        except Exception as e:
            self.get_logger().error(f'Error generating response: {e}')
            response.output = f'Error: {e}'
        return response


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = RosaServiceLifecycleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()