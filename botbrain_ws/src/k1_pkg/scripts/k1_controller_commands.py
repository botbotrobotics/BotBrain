#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

from unitree_api.msg import Request
from joystick_bot.msg import ControllerButtonsState
from unitree_go.msg import SportModeState
from bot_custom_interfaces.srv import Mode, Pose, BodyHeight, ContinuousGait, Euler, FootRaiseHeight, SwitchGait, SwitchJoystick, SpeedLevel
class OperationModes(LifecycleNode):

    def __init__(self):
 
        super().__init__('controller_commands_node')
        
        self.button_state_subscription = None
        self.sports_mode_subscription = None
        self.sport_publisher = None
        self.timer = None
        
        # Service Clients
        self.mode_srv_cli = None
        self.pose_srv_cli = None
        self.body_height_srv_cli = None
        self.continuous_gait_srv_cli = None
        self.euler_srv_cli = None
        self.foot_raise_height_srv_cli = None
        self.switch_gait_srv_cli = None
        self.switch_joystick_srv_cli = None
        self.speed_level_srv_cli = None
        
        # Internal state variables
        self.control_dict = {}
        self.button_states = {}
        self.op_mode = None
        self.gait_type = None
        
        self.get_logger().info("Lifecycle node created, in 'unconfigured' state.")

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        self.button_state_subscription = self.create_subscription(
            ControllerButtonsState,
            'button_state',
            self.button_subscription_callback,
            1)
        self.sports_mode_subscription = self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self.sport_state_subscription_callback,
            1)

        self.sport_publisher = self.create_publisher(Request, '/api/sport/request', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer.cancel() # Ensure timer doesn't start automatically

        # Create service clients
        self.mode_srv_cli = self.create_client(Mode, 'mode')
        self.pose_srv_cli = self.create_client(Pose, 'pose')
        self.body_height_srv_cli = self.create_client(BodyHeight, 'body_height')
        self.continuous_gait_srv_cli = self.create_client(ContinuousGait, 'continuous_gait')
        self.euler_srv_cli = self.create_client(Euler, 'euler')
        self.foot_raise_height_srv_cli = self.create_client(FootRaiseHeight, 'foot_raise_height')
        self.switch_gait_srv_cli = self.create_client(SwitchGait, 'switch_gait')
        self.switch_joystick_srv_cli = self.create_client(SwitchJoystick, 'switch_joystick')
        self.speed_level_srv_cli = self.create_client(SpeedLevel, 'speed_level')

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_activate() is called.")
        super().on_activate(state)
        self.timer.reset()
        
        self.get_logger().info("Node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_deactivate() is called.")
        super().on_deactivate(state)
        
        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_cleanup() is called.")
        
        # Destroy all ROS entities
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.sport_publisher)
        self.destroy_subscription(self.button_state_subscription)
        self.destroy_subscription(self.sports_mode_subscription)
        
        self.destroy_client(self.mode_srv_cli)
        self.destroy_client(self.pose_srv_cli)
        self.destroy_client(self.body_height_srv_cli)
        self.destroy_client(self.continuous_gait_srv_cli)
        self.destroy_client(self.euler_srv_cli)
        self.destroy_client(self.foot_raise_height_srv_cli)
        self.destroy_client(self.switch_gait_srv_cli)
        self.destroy_client(self.switch_joystick_srv_cli)
        self.destroy_client(self.speed_level_srv_cli)

        self.get_logger().info("Node cleaned up successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_shutdown() is called.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    
    def button_subscription_callback(self, msg: ControllerButtonsState):
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
    
    def sport_state_subscription_callback(self, msg: SportModeState):
        self.op_mode = msg.mode
        self.gait_type = msg.gait_type

    def timer_callback(self):

        if self.control_dict != self.button_states:
            f"Button state changed.\nOld: {self.control_dict}\nNew: {self.button_states}"
            self.get_logger().info(
                f"Button state changed.\nOld: {self.control_dict}\nNew: {self.button_states}"
            )
            # Mode Switch
            if self.button_states['start']:
                if self.op_mode == 2:
                    requ = Pose.Request()
                    requ.flag = False
                    self.pose_srv_cli.call_async(requ)
                else:
                    requ = Mode.Request()
                    requ.mode = 'balance_stand'
                    self.mode_srv_cli.call_async(requ)
                self.get_logger().info('start')
            if self.button_states['select']:
                requ = Pose.Request()
                requ.flag = True
                self.pose_srv_cli.call_async(requ)
                self.get_logger().info('select')
            if self.button_states['L2'] and self.button_states['A']:
                if self.op_mode == 7 or self.op_mode == 6:
                    requ = Mode.Request()
                    requ.mode = 'stand_down'
                    self.mode_srv_cli.call_async(requ)
                else:
                    requ = Mode.Request()
                    requ.mode = 'stand_up'
                    self.mode_srv_cli.call_async(requ)
                self.get_logger().info('L2+A')
            if self.button_states['L2'] and self.button_states['B']:
                requ = Mode.Request()
                requ.mode = 'damp'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('L2+B')
            if self.button_states['L2'] and self.button_states['start']:
                requ = SwitchGait.Request()
                requ.d = 2
                self.switch_gait_srv_cli.call_async(requ)
                self.get_logger().info('L2+start')
            if self.button_states['right'] and self.button_states['start']:
                requ = SwitchGait.Request()
                requ.d = 3
                self.switch_gait_srv_cli.call_async(requ)
                self.get_logger().info('right+start')
            if self.button_states['left'] and self.button_states['start']:
                requ = SwitchGait.Request()
                requ.d = 4
                self.switch_gait_srv_cli.call_async(requ)
                self.get_logger().info('left+start')
            if self.button_states['L1'] and self.button_states['select']:
                self.get_logger().info('L1+select')

            # Customized Movements
            if self.button_states['L2'] and self.button_states['X']:
                requ = Mode.Request()
                requ.mode = 'recovery_stand'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('L2+X')
            if self.button_states['R2'] and self.button_states['A']:
                requ = Mode.Request()
                requ.mode = 'stretch'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R2+A')
            if self.button_states['R2'] and self.button_states['B']:
                requ = Mode.Request()
                requ.mode = 'hello'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R2+B')
            if self.button_states['R2'] and self.button_states['Y']:
                requ = Mode.Request()
                requ.mode = 'finger_heart'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R2+Y')
            if self.button_states['R1'] and self.button_states['X']:
                requ = Mode.Request()
                requ.mode = 'front_pounce'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R1+X')
            if self.button_states['R1'] and self.button_states['A']:
                requ = Mode.Request()
                requ.mode = 'front_jump'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R1+A')
            if self.button_states['R1'] and self.button_states['B']:
                if self.op_mode == 10:
                    requ = Mode.Request()
                    requ.mode = 'rise_sit'
                    self.mode_srv_cli.call_async(requ)
                else:
                    requ = Mode.Request()
                    requ.mode = 'sit'
                    self.mode_srv_cli.call_async(requ)
                self.get_logger().info('R1+B')
            if self.button_states['L1'] and self.button_states['A']:
                self.get_logger().info('L1+A')
            if self.button_states['L1'] and self.button_states['B']:
                requ = Mode.Request()
                requ.mode = 'dance1'
                self.mode_srv_cli.call_async(requ)
                self.get_logger().info('L1+B')
                
            # After processing commands, update the stored state
            self.control_dict = self.button_states.copy()


def main(args=None):
    rclpy.init(args=args)
    node = OperationModes()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()