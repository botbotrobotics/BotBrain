#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped, TransformStamped

class MapOdomPosePublisher(LifecycleNode):
    """
    A lifecycle node that listens to the TF transform from 'map' to 'base_link'
    and publishes it as a PoseStamped message.
    """
    def __init__(self):
        # Initialize the lifecycle node
        super().__init__('map_odom_pose_publisher')
        
        # Initialize resources to None. They will be created in on_configure.
        self.pose_publisher = None
        self.tf_buffer = None
        self.tf_listener = None
        self.timer = None
        
        self.prefix = self.declare_parameter("prefix", "").get_parameter_value().string_value

        self.get_logger().info("Lifecycle node created. Waiting for configuration...")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to 'inactive': Configuring node...")
        
        self.pose_publisher = self.create_publisher(PoseStamped, 'map_odom', 10)
        
        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to 'active': Activating node...")

        # Required by the base class
        super().on_activate(state)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Start the timer to begin publishing
        self.timer = self.create_timer(0.1, self.publish_map_odom_pose)
        
        self.get_logger().info("Node activated. Publishing pose now.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to 'inactive': Deactivating node...")
        
        # Stop publishing by destroying the timer
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        
        # Required by the base class
        super().on_deactivate(state)

         # TF listener doesn't have an explicit destroy method
        self.tf_listener = None
        self.tf_buffer = None

        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Transitioning to 'unconfigured': Cleaning up...")

        if self.pose_publisher:
            self.destroy_publisher(self.pose_publisher)
            self.pose_publisher = None

        self.get_logger().info("Cleanup complete.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Node is shutting down. Releasing resources...")
        
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        
         # TF listener doesn't have an explicit destroy method
        self.tf_listener = None
        self.tf_buffer = None

        if self.pose_publisher:
            self.destroy_publisher(self.pose_publisher)
            self.pose_publisher = None

        return TransitionCallbackReturn.SUCCESS
    
    ##
    ## NODE'S CORE LOGIC
    ##

    def publish_map_odom_pose(self):
        """
        Looks up the transform and publishes it as a pose. This method only runs
        when the node is in the 'active' state.
        """
        from_frame = f'{self.prefix}map'
        to_frame = f'{self.prefix}base_link'
        
        try:
            # Look up the transform from map to base_link
            t_stamped: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rclpy.time.Time()
            )
            
            # Convert the transform to a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = from_frame
            pose_msg.pose.position.x = t_stamped.transform.translation.x
            pose_msg.pose.position.y = t_stamped.transform.translation.y
            pose_msg.pose.position.z = t_stamped.transform.translation.z
            pose_msg.pose.orientation = t_stamped.transform.rotation
            
            # Publish the pose
            self.pose_publisher.publish(pose_msg)
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {from_frame} to {to_frame}: {ex}',
                throttle_duration_sec=1.0 # Avoid spamming warnings
            )

def main(args=None):
    rclpy.init(args=args)

    # Use a SingleThreadedExecutor to handle the node's lifecycle
    executor = rclpy.executors.SingleThreadedExecutor()
    node = MapOdomPosePublisher()
    executor.add_node(node)
    
    try:
        # Spin the executor to process callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    # The node will be properly shutdown by the context manager
    rclpy.shutdown()

if __name__ == '__main__':
    main()