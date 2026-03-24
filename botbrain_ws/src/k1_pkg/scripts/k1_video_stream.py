#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class VideoPublisher(LifecycleNode):

    def __init__(self):
        super().__init__('robot_video_stream')

        # Declare parameters
        self.declare_parameter('network_interface', 'eno1')

        # Initialize members to None. They will be populated in on_configure.
        self.publisher_img = None
        self.publisher_compressed = None
        self.timer = None
        self.cap = None
        self.bridge = CvBridge()
        self.gstreamer_str = None
        self.get_logger().info("Lifecycle node created. Awaiting configuration...")

    # --- Lifecycle Transition Callbacks ---

    def on_configure(self, state):
        self.get_logger().info('In on_configure, configuring the node...')
        try:
            # Get network interface parameter
            network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
            self.get_logger().info(f'Using network interface: {network_interface}')

            # Define QoS profile for the compressed image publisher
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            # Create publishers
            self.publisher_img = self.create_publisher(Image, 'go2_camera', 1)
            self.publisher_compressed = self.create_publisher(CompressedImage, 'go2_compressed_camera', qos_profile)

            # Prepare the GStreamer pipeline string (but don't open it yet)
            self.gstreamer_str = f"udpsrc address=230.1.1.1 port=1720 multicast-iface={network_interface} ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"

            self.get_logger().info('Configuration successful.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Error during configuration: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('In on_activate, activating the node...')
        try:
            # Start the timer to publish frames
            # The GStreamer pipeline will be opened on the first timer callback
            rate = 1.0 / 24.0
            self.timer = self.create_timer(rate, self.timer_callback)
            self.get_logger().info('Node activated. Video stream will start on first frame.')
            return super().on_activate(state)

        except Exception as e:
            self.get_logger().error(f'Error during activation: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        self.get_logger().info('In on_deactivate, deactivating the node...')
        # Stop publishing by destroying the timer
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        # Release the video capture
        if self.cap:
            self.cap.release()
            self.cap = None
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
        """Helper method to destroy publishers and release the camera."""
        if self.timer:
            self.destroy_timer(self.timer)
        if self.publisher_img:
            self.destroy_publisher(self.publisher_img)
        if self.publisher_compressed:
            self.destroy_publisher(self.publisher_compressed)
        if self.cap:
            self.cap.release()
        
        # Reset members
        self.timer = None
        self.publisher_img = None
        self.publisher_compressed = None
        self.cap = None

    def timer_callback(self):
        """Callback to capture a frame and publish it."""
        # Lazy initialization: open the pipeline on first callback
        if self.cap is None:
            self.get_logger().info('Opening GStreamer pipeline...')
            self.cap = cv2.VideoCapture(self.gstreamer_str, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open GStreamer pipeline.')
                return
            self.get_logger().info('GStreamer pipeline opened successfully.')

        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.get_logger().debug('Publishing video frame')
                now = self.get_clock().now().to_msg()

                # Publish raw image
                raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                raw_msg.header.stamp = now
                self.publisher_img.publish(raw_msg)

                # Publish compressed image
                small_frame = cv2.resize(frame, (640, 360))
                ret_enc, jpeg = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])
                if ret_enc:
                    comp_msg = CompressedImage()
                    comp_msg.header.stamp = now
                    comp_msg.format = "jpeg"
                    comp_msg.data = jpeg.tobytes()
                    self.publisher_compressed.publish(comp_msg)
            else:
                self.get_logger().warn('Failed to read frame from camera.')


def main(args=None):
    rclpy.init(args=args)
    lifecycle_video_publisher = VideoPublisher()
    rclpy.spin(lifecycle_video_publisher)
    lifecycle_video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()