#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


def _load_robot_model():
    """Read robot_model from robot_config.yaml, located 4 levels above the installed script."""
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        config_file = os.path.join(workspace_dir, 'robot_config.yaml')
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)['robot_configuration']['robot_model']
    except Exception:
        return 'realsense'


class RealsenseCompressedNode(LifecycleNode):

    def __init__(self):
        super().__init__('realsense_compressed_node')

        self.robot_model = _load_robot_model()

        self.publisher_compressed = None
        self.publisher_compressed_back = None
        self.subscription = None
        self.subscription_back = None
        self.bridge = CvBridge()
        self.get_logger().info(f"Lifecycle node created (robot_model='{self.robot_model}'). Awaiting configuration...")

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
            # Create publisher for front/K1 camera compressed image
            self.publisher_compressed = self.create_publisher(CompressedImage, 'compressed_camera', qos_profile)
            # Back camera publisher (realsense) or depth publisher (K1)
            self.publisher_compressed_back = self.create_publisher(CompressedImage, 'compressed_back_camera', qos_profile)

        except Exception as e:
            self.get_logger().error(f'Error during configuration: {e}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Configuration successful.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('In on_activate, activating the node...')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        if self.robot_model == 'k1':
            # K1 publishes /rgb/image with BEST_EFFORT (qos_profile_sensor_data)
            k1_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.subscription = self.create_subscription(
                Image,
                '/rgb/image',
                self.image_callback,
                k1_qos
            )
            # K1 has no back camera — use depth image instead
            self.subscription_back = self.create_subscription(
                Image,
                '/depth/visual',
                self.depth_callback,
                k1_qos
            )
            self.get_logger().info('Node activated and subscribed to /rgb/image and /depth/image (K1 stereo camera).')
        else:
            # Create subscription to RealSense image topics
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
        self._cleanup_resources()
        self.get_logger().info('Cleanup successful.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('In on_shutdown, shutting down the node...')
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

    def depth_callback(self, msg):
        """Callback to process K1 depth image and publish as compressed colormap (used as back camera slot)."""
        try:
            self.get_logger().debug('Received and processing K1 depth image frame')

            if msg.encoding in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
                # stereonet_visual: top half = raw, bottom half = depth colormap — crop bottom only
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                frame = frame[frame.shape[0] // 2:, :]
                small_frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
                ret_enc, jpeg = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])
            else:
                # Raw depth (32FC1 / 16UC1): normalize and apply colormap
                import numpy as np
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if frame.dtype != np.float32:
                    frame = frame.astype(np.float32)
                valid = frame[np.isfinite(frame)]
                if valid.size == 0:
                    return
                d_min, d_max = valid.min(), valid.max()
                if d_max - d_min < 1e-6:
                    return
                normalized = np.clip((frame - d_min) / (d_max - d_min), 0.0, 1.0)
                gray8 = (normalized * 255).astype(np.uint8)
                colored = cv2.applyColorMap(gray8, cv2.COLORMAP_INFERNO)
                small_frame = cv2.resize(colored, (640, 360), interpolation=cv2.INTER_AREA)
                ret_enc, jpeg = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])

            if ret_enc:
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = "jpeg"
                comp_msg.data = jpeg.tobytes()
                self.publisher_compressed_back.publish(comp_msg)
                self.get_logger().debug('Published compressed K1 depth image')
            else:
                self.get_logger().warn('Failed to encode K1 depth image to JPEG')

        except Exception as e:
            self.get_logger().error(f'Error processing K1 depth image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCompressedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
