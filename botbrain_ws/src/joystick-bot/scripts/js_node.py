#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.executors import MultiThreadedExecutor
from joystick_bot.msg import ControllerButtonsState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pygame
import time
import os

class JoyStickNode(LifecycleNode):
    def __init__(self):
        super().__init__('joystick_interface')
        

        self.cmd_pub = None
        self.buttons_pub = None
        self.dead_man_pub = None
        self.timer = None
        self.joystick = None

        self.declare_parameter('device_input', '/dev/input/js0')
        self.declare_parameter('linear_deadband', 0.02)
        self.declare_parameter('angular_deadband', 0.02)

        # Topic name parameters
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_joy')
        self.declare_parameter('button_state_topic', 'button_state')
        self.declare_parameter('dead_man_switch_topic', 'dead_man_switch')

        # Button mapping parameters
        self.declare_parameter('button_mapping.x_button', 0)
        self.declare_parameter('button_mapping.a_button', 1)
        self.declare_parameter('button_mapping.b_button', 2)
        self.declare_parameter('button_mapping.y_button', 3)
        self.declare_parameter('button_mapping.l1_button', 4)
        self.declare_parameter('button_mapping.r1_button', 5)
        self.declare_parameter('button_mapping.l2_button', 6)
        self.declare_parameter('button_mapping.r2_button', 7)
        self.declare_parameter('button_mapping.select_button', 8)
        self.declare_parameter('button_mapping.start_button', 9)
        self.declare_parameter('button_mapping.dead_man_button', 4)

        # Axis mapping parameters
        self.declare_parameter('axis_mapping.linear_x_axis', 1)
        self.declare_parameter('axis_mapping.linear_y_axis', 0)
        self.declare_parameter('axis_mapping.angular_y_axis', 5)
        self.declare_parameter('axis_mapping.angular_z_axis', 2)

        self.device_input = ''
        self.linear_deadband = 0.0
        self.angular_deadband = 0.0
        self.cmd_vel_topic = ''
        self.button_state_topic = ''
        self.dead_man_switch_topic = ''
        self.button_map = {}
        self.axis_map = {}

        self.joystick_connected = False

        self.get_logger().info(f"Node '{self.get_name()}' created and in 'unconfigured' state.")


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In on_configure()")

        # Get parameter values
        self.device_input = self.get_parameter('device_input').get_parameter_value().string_value
        self.linear_deadband = self.get_parameter('linear_deadband').get_parameter_value().double_value
        self.angular_deadband = self.get_parameter('angular_deadband').get_parameter_value().double_value

        # Get topic names
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.button_state_topic = self.get_parameter('button_state_topic').get_parameter_value().string_value
        self.dead_man_switch_topic = self.get_parameter('dead_man_switch_topic').get_parameter_value().string_value

        # Get button mappings
        self.button_map = {
            'x_button': self.get_parameter('button_mapping.x_button').get_parameter_value().integer_value,
            'a_button': self.get_parameter('button_mapping.a_button').get_parameter_value().integer_value,
            'b_button': self.get_parameter('button_mapping.b_button').get_parameter_value().integer_value,
            'y_button': self.get_parameter('button_mapping.y_button').get_parameter_value().integer_value,
            'l1_button': self.get_parameter('button_mapping.l1_button').get_parameter_value().integer_value,
            'r1_button': self.get_parameter('button_mapping.r1_button').get_parameter_value().integer_value,
            'l2_button': self.get_parameter('button_mapping.l2_button').get_parameter_value().integer_value,
            'r2_button': self.get_parameter('button_mapping.r2_button').get_parameter_value().integer_value,
            'select_button': self.get_parameter('button_mapping.select_button').get_parameter_value().integer_value,
            'start_button': self.get_parameter('button_mapping.start_button').get_parameter_value().integer_value,
            'dead_man_button': self.get_parameter('button_mapping.dead_man_button').get_parameter_value().integer_value,
        }

        # Get axis mappings
        self.axis_map = {
            'linear_x': self.get_parameter('axis_mapping.linear_x_axis').get_parameter_value().integer_value,
            'linear_y': self.get_parameter('axis_mapping.linear_y_axis').get_parameter_value().integer_value,
            'angular_y': self.get_parameter('axis_mapping.angular_y_axis').get_parameter_value().integer_value,
            'angular_z': self.get_parameter('axis_mapping.angular_z_axis').get_parameter_value().integer_value,
        }

        # Create publishers with configurable topic names
        self.cmd_pub = self.create_lifecycle_publisher(Twist, self.cmd_vel_topic, 10)
        self.buttons_pub = self.create_lifecycle_publisher(ControllerButtonsState, self.button_state_topic, 10)
        self.dead_man_pub = self.create_lifecycle_publisher(Bool, self.dead_man_switch_topic, 10)

        self.get_logger().info("Configuration successful.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In on_activate()")
        try:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            os.environ["SDL_JOYSTICK_DEVICE"] = self.device_input
            pygame.display.init()
            pygame.joystick.init()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize pygame: {e}")
            return TransitionCallbackReturn.ERROR
        
        self.timer = self.create_timer(0.05, self.check_for_events)
        super().on_activate(state)
        self.get_logger().info("Node is active and polling has started.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In on_deactivate()")

        # Stop polling
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        # Publish a safety stop once if we were active
        if self.cmd_pub is not None and getattr(self.cmd_pub, "is_activated", False):
            try:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Safety stop command published.")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish safety stop: {e}")

        # Deactivate lifecycle publishers (guarded if they weren't activated)
        try:
            if self.cmd_pub is not None and getattr(self.cmd_pub, "is_activated", False):
                self.cmd_pub.on_deactivate()
            if self.buttons_pub is not None and getattr(self.buttons_pub, "is_activated", False):
                self.buttons_pub.on_deactivate()
            if self.dead_man_pub is not None and getattr(self.dead_man_pub, "is_activated", False):
                self.dead_man_pub.on_deactivate()
        except Exception as e:
            self.get_logger().warn(f"Publisher deactivation warning: {e}")

        # Keep pygame modules initialized during Inactive state so hot-plug events
        # will still be received on re-activate. Do NOT call pygame.joystick.quit() here.
        self.get_logger().info("Node is inactive.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In on_cleanup()")

        # Ensure timer is stopped
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        # Release the joystick device handle, but only quit the *instance* here
        if self.joystick is not None:
            try:
                # Close just this device; safe even if already closed
                self.joystick.quit()
            except Exception:
                pass
            finally:
                self.joystick = None

        # Destroy publishers
        if self.cmd_pub is not None:
            try:
                self.destroy_publisher(self.cmd_pub)
            except Exception:
                pass
            self.cmd_pub = None

        if self.buttons_pub is not None:
            try:
                self.destroy_publisher(self.buttons_pub)
            except Exception:
                pass
            self.buttons_pub = None

        if self.dead_man_pub is not None:
            try:
                self.destroy_publisher(self.dead_man_pub)
            except Exception:
                pass
            self.dead_man_pub = None

        # Now that we're cleaning up, it *is* appropriate to tear down pygame modules
        try:
            if pygame.joystick.get_init():
                pygame.joystick.quit()
            if pygame.display.get_init():
                pygame.display.quit()
            if pygame.get_init():
                pygame.quit()
        except Exception as e:
            self.get_logger().warn(f"Pygame cleanup warning: {e}")

        self.get_logger().info("Cleanup successful.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In on_shutdown()")
        return self.on_cleanup(state)

    def connect_joystick(self) -> bool:
        if rclpy.ok():
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Joystick '{self.joystick.get_name()}' connected.")
                return True
            self.get_logger().info("No joystick found. Retrying in 1 second...")
        return False 

    def check_for_events(self):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYDEVICEREMOVED:
                #pygame.joystick.quit()
                self.joystick = None
                self.get_logger().info("Joystick disconnected.")
            if event.type == pygame.JOYDEVICEADDED and self.joystick is None:
                self.connect_joystick()
            if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYHATMOTION:
                self.publish_button_state()
        if self.joystick:
            self.publish_twist()

    def publish_button_state(self):
        if not self.buttons_pub.is_activated:
            self.get_logger().error("CRITICAL: Button publisher is not active!")
            return

        buttons_msg = ControllerButtonsState()

        # Use configurable button mappings
        buttons_msg.x_button = bool(self.joystick.get_button(self.button_map['x_button']))
        buttons_msg.a_button = bool(self.joystick.get_button(self.button_map['a_button']))
        buttons_msg.b_button = bool(self.joystick.get_button(self.button_map['b_button']))
        buttons_msg.y_button = bool(self.joystick.get_button(self.button_map['y_button']))
        buttons_msg.l1_button = bool(self.joystick.get_button(self.button_map['l1_button']))
        buttons_msg.r1_button = bool(self.joystick.get_button(self.button_map['r1_button']))
        buttons_msg.l2_button = bool(self.joystick.get_button(self.button_map['l2_button']))
        buttons_msg.r2_button = bool(self.joystick.get_button(self.button_map['r2_button']))
        buttons_msg.select_button = bool(self.joystick.get_button(self.button_map['select_button']))
        buttons_msg.start_button = bool(self.joystick.get_button(self.button_map['start_button']))

        hat = self.joystick.get_hat(0)
        buttons_msg.right_button = hat[0] == 1
        buttons_msg.up_button = hat[1] == 1
        buttons_msg.left_button = hat[0] == -1
        buttons_msg.down_button = hat[1] == -1

        self.buttons_pub.publish(buttons_msg)

        dead_man = Bool()
        dead_man.data = bool(self.joystick.get_button(self.button_map['dead_man_button']))
        self.dead_man_pub.publish(dead_man)

    def publish_twist(self):
        if not self.cmd_pub.is_activated:
            self.get_logger().error("CRITICAL: Twist publisher is not active!")
            return

        twist_msg = Twist()

        # Use configurable axis mappings
        linear_x_value = self.joystick.get_axis(self.axis_map['linear_x'])
        linear_y_value = self.joystick.get_axis(self.axis_map['linear_y'])
        angular_y_value = self.joystick.get_axis(self.axis_map['angular_y'])
        angular_z_value = self.joystick.get_axis(self.axis_map['angular_z'])

        twist_msg.linear.x = -linear_x_value if abs(linear_x_value) > self.linear_deadband else 0.0
        twist_msg.linear.y = -linear_y_value if abs(linear_y_value) > self.linear_deadband else 0.0
        twist_msg.angular.y = -angular_y_value if abs(angular_y_value) > self.angular_deadband else 0.0
        twist_msg.angular.z = -angular_z_value if abs(angular_z_value) > self.angular_deadband else 0.0

        if twist_msg.linear.x != 0 or twist_msg.linear.y != 0 or twist_msg.angular.z != 0 or twist_msg.angular.y != 0:
            self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = JoyStickNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()