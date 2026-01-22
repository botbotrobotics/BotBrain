#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class RealsenseCompressedNode(LifecycleNode):

    def __init__(self):
        super().__init__('realsense_compressed_node')

        self.publisher_compressed = None
        self.publisher_compressed_back = None
        self.subscription = None
        self.subscription_back = None
        self.bridge = CvBridge()
        self.get_logger().info("Lifecycle node created. Awaiting configuration...")

    # --- Lifecycle Transition Callbacks ---

    def on_configure(self, state):
        self.get_logger().info('In on_configure, configuring the node...')
        try:
            # Define QoS profile for the compressed image publisher
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            # Create publisher for front camera compressed image
            self.publisher_compressed = self.create_publisher(CompressedImage, 'compressed_camera', qos_profile)
            # Create publisher for back camera compressed image
            self.publisher_compressed_back = self.create_publisher(CompressedImage, 'compressed_back_camera', qos_profile)

        except Exception as e:
            self.get_logger().error(f'Error during configuration: {e}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Configuration successful.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('In on_activate, activating the node...')
        # Create subscription to RealSense image topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image,
            'front_camera/color/image_raw',
            self.image_callback,
            qos_profile
        )
        self.subscription_back = self.create_subscription(
            Image,
            '/back_camera/color/image_raw',
            self.image_callback_back,
            qos_profile
        )
        self.get_logger().info('Node activated and subscribed to front and back camera topics.')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('In on_deactivate, deactivating the node...')
        # Stop subscribing by destroying the subscriptions
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        if self.subscription_back:
            self.destroy_subscription(self.subscription_back)
            self.subscription_back = None
        self.get_logger().info('Node deactivated.')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('In on_cleanup, cleaning up resources...')
        # Release all resources
        self._cleanup_resources()
        self.get_logger().info('Cleanup successful.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('In on_shutdown, shutting down the node...')
        # Ensure all resources are released on shutdown
        self._cleanup_resources()
        self.get_logger().info('Shutdown complete.')
        return TransitionCallbackReturn.SUCCESS


    def _cleanup_resources(self):
        """Helper method to destroy publishers and subscriptions."""
        if self.subscription:
            self.destroy_subscription(self.subscription)
        if self.subscription_back:
            self.destroy_subscription(self.subscription_back)
        if self.publisher_compressed:
            self.destroy_publisher(self.publisher_compressed)
        if self.publisher_compressed_back:
            self.destroy_publisher(self.publisher_compressed_back)

        # Reset members
        self.subscription = None
        self.subscription_back = None
        self.publisher_compressed = None
        self.publisher_compressed_back = None

    def image_callback(self, msg):
        """Callback to process received front camera image and publish compressed version."""
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.get_logger().debug('Received and processing front camera image frame')

            # Resize and compress the image
            small_frame = cv2.resize(frame, (640, 360))
            ret_enc, jpeg = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])

            if ret_enc:
                # Create compressed image message
                comp_msg = CompressedImage()
                comp_msg.header = msg.header  # Keep original timestamp and frame_id
                comp_msg.format = "jpeg"
                comp_msg.data = jpeg.tobytes()
                self.publisher_compressed.publish(comp_msg)
                self.get_logger().debug('Published compressed front camera image')
            else:
                self.get_logger().warn('Failed to encode front camera image to JPEG')

        except Exception as e:
            self.get_logger().error(f'Error processing front camera image: {e}')

    def image_callback_back(self, msg):
        """Callback to process received back camera image and publish compressed version."""
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.get_logger().debug('Received and processing back camera image frame')

            # Resize and compress the image
            small_frame = cv2.resize(frame, (640, 360))
            ret_enc, jpeg = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])

            if ret_enc:
                # Create compressed image message
                comp_msg = CompressedImage()
                comp_msg.header = msg.header  # Keep original timestamp and frame_id
                comp_msg.format = "jpeg"
                comp_msg.data = jpeg.tobytes()
                self.publisher_compressed_back.publish(comp_msg)
                self.get_logger().debug('Published compressed back camera image')
            else:
                self.get_logger().warn('Failed to encode back camera image to JPEG')

        except Exception as e:
            self.get_logger().error(f'Error processing back camera image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCompressedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()