#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, JointState, BatteryState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class RobotRead(LifecycleNode):
    def __init__(self):
        super().__init__('robot_read_node')
        self.prefix = ''
        self.tita_namespace = ''

        self.odometry_subscriber = None
        self.joint_state_subscriber = None
        self.imu_subscriber = None
        self.point_cloud_subscriber = None
        self.camera_subscriber = None

        self.tf_broadcaster = None
        self.odom_pub = None
        self.imu_pub = None
        self.camera_pub = None
        self.pointcloud_pub = None
        self.joint_state_pub = None
        self.left_battery_state = None
        self.right_battery_state = None
        self.battery_state_pub = None
        self.battery_timer = None

        self.get_logger().info('Lifecycle node created, currently in Unconfigured state.')

    def on_configure(self, state):
        self.get_logger().info('on_configure() is called.')
        self.declare_parameter('prefix', '')
        self.declare_parameter('tita_namespace', '')
        self.prefix = self.get_parameter('prefix').value
        self.tita_namespace = self.get_parameter('tita_namespace').value

        # Build namespace prefix with proper slash handling
        namespace_prefix = f'/{self.tita_namespace}' if self.tita_namespace else ''

        self.odometry_subscriber = self.create_subscription(Odometry, f'{namespace_prefix}/chassis/odometry', self.odometry_callback, 10)
        self.joint_state_subscriber = self.create_subscription(JointState, f'{namespace_prefix}/joint_states', self.joint_state_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, f'{namespace_prefix}/imu_sensor_broadcaster', self.imu_callback, 10)
        self.point_cloud_subscriber = self.create_subscription(PointCloud2, f'{namespace_prefix}/perception/camera/point_cloud', self.point_cloud_callback, 10)
        self.camera_subscriber = self.create_subscription(Image, f'{namespace_prefix}/perception/camera/image/raw', self.camera_callback, 10)
        self.left_battery_subscriber = self.create_subscription(BatteryState, f'{namespace_prefix}/system/battery/left', self.left_battery_callback, 10)
        self.right_battery_subscriber = self.create_subscription(BatteryState, f'{namespace_prefix}/system/battery/right', self.right_battery_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_lifecycle_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_lifecycle_publisher(Imu, 'imu/data', 10)
        self.camera_pub = self.create_lifecycle_publisher(Image, 'camera/image', 10)
        self.pointcloud_pub = self.create_lifecycle_publisher(PointCloud2, 'pointcloud', 10)
        self.joint_state_pub = self.create_lifecycle_publisher(JointState, 'joint_states', 10)
        self.battery_state_pub = self.create_lifecycle_publisher(BatteryState, 'battery', 10)

        self.get_logger().info('Node configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate() is called.')
        super().on_activate(state)
        self.battery_timer = self.create_timer(1.0, self.battery_timer_callback)
        self.get_logger().info('Node activated.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate() is called.')
        super().on_deactivate(state)
        if self.battery_timer:
            self.destroy_timer(self.battery_timer)
            self.battery_timer = None
        self.get_logger().info('Node deactivated.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup() is called.')
        self.destroy_subscription(self.odometry_subscriber)
        self.destroy_subscription(self.joint_state_subscriber)
        self.destroy_subscription(self.imu_subscriber)
        self.destroy_subscription(self.point_cloud_subscriber)
        self.destroy_subscription(self.camera_subscriber)
        
        self.destroy_publisher(self.odom_pub)
        self.destroy_publisher(self.imu_pub)
        self.destroy_publisher(self.camera_pub)
        self.destroy_publisher(self.pointcloud_pub)
        self.destroy_publisher(self.joint_state_pub)
        self.destroy_publisher(self.battery_state_pub)
        
        self.get_logger().info('Node cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    # --- Regular Callbacks (Corrected: 'if active' check removed) ---
    
    def odometry_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = f'{self.prefix}odom'
        transform.child_frame_id = f'{self.prefix}base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

        odom = msg
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = f'{self.prefix}odom'
        odom.child_frame_id = f'{self.prefix}base_link'
        self.odom_pub.publish(odom)

    def imu_callback(self, msg):
        imu = msg
        imu.header.stamp = self.get_clock().now().to_msg() 
        imu.header.frame_id = f'{self.prefix}imu_link'
        self.imu_pub.publish(imu)

    def camera_callback(self, msg):
        camera = msg
        camera.header.stamp = self.get_clock().now().to_msg() 
        camera.header.frame_id = f'{self.prefix}right_camera'
        self.camera_pub.publish(camera)

    def point_cloud_callback(self, msg):
        lidar = msg
        lidar.header.stamp = self.get_clock().now().to_msg() 
        lidar.header.frame_id = f'{self.prefix}left_camera'
        self.pointcloud_pub.publish(lidar)

    def joint_state_callback(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            f'{self.prefix}joint_left_leg_1', f'{self.prefix}joint_left_leg_2',
            f'{self.prefix}joint_left_leg_3', f'{self.prefix}joint_left_leg_4',
            f'{self.prefix}joint_right_leg_1', f'{self.prefix}joint_right_leg_2',
            f'{self.prefix}joint_right_leg_3', f'{self.prefix}joint_right_leg_4'
        ]
        joint_state_msg.position = msg.position
        joint_state_msg.velocity = msg.velocity
        joint_state_msg.effort = msg.effort
        self.joint_state_pub.publish(joint_state_msg)

    def left_battery_callback(self, msg):
        self.left_battery_state = msg

    def right_battery_callback(self, msg):
        self.right_battery_state = msg

    def battery_timer_callback(self):
        if self.left_battery_state is not None and self.right_battery_state is not None:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.header.frame_id = f'{self.prefix}base_link'
            
            battery_msg.voltage = (self.left_battery_state.voltage + self.right_battery_state.voltage) / 2.0
            battery_msg.percentage = (self.left_battery_state.percentage + self.right_battery_state.percentage) / 2.0 / 100.0
            
            battery_msg.temperature = max(self.left_battery_state.temperature, self.right_battery_state.temperature)
            battery_msg.current = self.left_battery_state.current + self.right_battery_state.current
            battery_msg.charge = self.left_battery_state.charge + self.right_battery_state.charge
            battery_msg.capacity = self.left_battery_state.capacity + self.right_battery_state.capacity
            battery_msg.design_capacity = self.left_battery_state.design_capacity + self.right_battery_state.design_capacity
            
            if self.left_battery_state.power_supply_status == self.right_battery_state.power_supply_status:
                battery_msg.power_supply_status = self.left_battery_state.power_supply_status
            else:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            
            if self.left_battery_state.power_supply_health == self.right_battery_state.power_supply_health:
                battery_msg.power_supply_health = self.left_battery_state.power_supply_health
            else:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                
            battery_msg.power_supply_technology = self.left_battery_state.power_supply_technology
            battery_msg.present = self.left_battery_state.present and self.right_battery_state.present
            
            self.battery_state_pub.publish(battery_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRead()
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