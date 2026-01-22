#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose, FollowWaypoints


class Nav2Utils(LifecycleNode):
    def __init__(self):
        super().__init__('nav2_utils')

        self._cancel_nav_client = None
        self._cancel_waypoints_client = None
        self._cancel_service = None
        self._callback_group = ReentrantCallbackGroup()

        self.get_logger().info('Nav2 Utils lifecycle node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configuring state callback
        """
        self.get_logger().info('Configuring Nav2 Utils node')

        # Create service clients to cancel action goals
        self._cancel_nav_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
            callback_group=self._callback_group
        )

        self._cancel_waypoints_client = self.create_client(
            CancelGoal,
            '/follow_waypoints/_action/cancel_goal',
            callback_group=self._callback_group
        )

        # Create service to cancel Nav2 goal
        self._cancel_service = self.create_service(
            Trigger,
            '/cancel_nav2_goal',
            self.cancel_goal_callback,
            callback_group=self._callback_group
        )

        self.get_logger().info('Nav2 Utils node configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activating state callback
        """
        self.get_logger().info('Activating Nav2 Utils node')
        self.get_logger().info('Service /cancel_nav2_goal is active and ready')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Deactivating state callback
        """
        self.get_logger().info('Deactivating Nav2 Utils node')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Cleanup state callback
        """
        self.get_logger().info('Cleaning up Nav2 Utils node')

        if self._cancel_service:
            self.destroy_service(self._cancel_service)
            self._cancel_service = None

        if self._cancel_nav_client:
            self.destroy_client(self._cancel_nav_client)
            self._cancel_nav_client = None

        if self._cancel_waypoints_client:
            self.destroy_client(self._cancel_waypoints_client)
            self._cancel_waypoints_client = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Shutdown state callback
        """
        self.get_logger().info('Shutting down Nav2 Utils node')
        return TransitionCallbackReturn.SUCCESS

    def cancel_goal_callback(self, request, response):
        """
        Service callback to cancel the current Nav2 goal (both NavigateToPose and FollowWaypoints)
        """
        goal_cancelled = False

        # Cancel NavigateToPose goals
        if self._cancel_nav_client.service_is_ready():
            try:
                cancel_request = CancelGoal.Request()
                result = self._cancel_nav_client.call(cancel_request)

                if result and len(result.goals_canceling) > 0:
                    goal_cancelled = True
                    self.get_logger().info('NavigateToPose goal cancelled')

            except Exception as e:
                self.get_logger().error(f'Error canceling NavigateToPose goal: {str(e)}')

        # Cancel FollowWaypoints goals
        if self._cancel_waypoints_client.service_is_ready():
            try:
                cancel_request = CancelGoal.Request()
                result = self._cancel_waypoints_client.call(cancel_request)

                if result and len(result.goals_canceling) > 0:
                    goal_cancelled = True
                    self.get_logger().info('FollowWaypoints goal cancelled')

            except Exception as e:
                self.get_logger().error(f'Error canceling FollowWaypoints goal: {str(e)}')

        # Build response
        if goal_cancelled:
            response.success = True
            response.message = 'Navigation goal cancelled'
            self.get_logger().info('Navigation goal cancelled successfully')
        else:
            response.success = True
            response.message = 'No active navigation goals to cancel'
            self.get_logger().info('No active navigation goals found')

        return response


def main(args=None):
    rclpy.init(args=args)

    nav2_utils_node = Nav2Utils()
    executor = MultiThreadedExecutor()
    executor.add_node(nav2_utils_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nav2_utils_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
