#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        
        # Declare parameters with default values
        self.declare_parameter('hostname', '192.168.1.1')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')
        self.declare_parameter('port', 554)
        self.declare_parameter('stream', 'Streaming/Channels/102')
        self.declare_parameter('inverted', False)
        self.declare_parameter('publish_rate', 30)
        
        # Get parameters
        self.hostname = self.get_parameter('hostname').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.port = self.get_parameter('port').value
        self.stream = self.get_parameter('stream').value
        self.inverted = self.get_parameter('inverted').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Print all parameters
        self.get_logger().info("=" * 60)
        self.get_logger().info("Camera RTSP Streamer Node - Configuration Parameters")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Hostname: {self.hostname}")
        self.get_logger().info(f"  Username: {self.username}")
        self.get_logger().info(f"  Password: {'***' if self.password else '(empty)'}")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info(f"  Stream: {self.stream}")
        self.get_logger().info(f"  Inverted: {self.inverted}")
        self.get_logger().info(f"  Publish Rate: {self.publish_rate} Hz")
        safe_url = f'rtsp://{self.username}:***@{self.hostname}:{self.port}/{self.stream}'
        self.get_logger().info(f"  RTSP URL: {safe_url}")
        self.get_logger().info("=" * 60)
        
        # Construct RTSP URL
        self.rtsp_url = f'rtsp://{self.username}:{self.password}@{self.hostname}:{self.port}/{self.stream}'
        
        # Create publisher
        self.publisher = self.create_publisher(Image, 'rtsp_stream', 10)
        
        # Create CvBridge object
        self.bridge = CvBridge()
        
        # Initialize video capture
        self.cap = None
        self.initialize_camera()
        
        # Create timer for publishing frames
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_frame)
        

    def initialize_camera(self):
        """Initialize the camera connection"""
        if self.cap is not None:
            self.cap.release()
        
        self.cap = cv2.VideoCapture(self.rtsp_url)
        
        if not self.cap.isOpened():
            safe_url = f'rtsp://{self.username}:***@{self.hostname}:{self.port}/{self.stream}'
            self.get_logger().error(f"Error opening video stream: {safe_url}")
            return False
        
        self.get_logger().info("Camera connection established successfully")
        return True

    def publish_frame(self):
        """Publish a single frame from the RTSP stream"""
        if self.cap is None or not self.cap.isOpened():
            if not self.initialize_camera():
                return
        
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warning("Can't receive frame. Retrying...")
            return

        # Rotate 180 degrees if inverted flag is set
        if self.inverted:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_link"
            self.publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

    def destroy_node(self):
        """Cleanup resources when node is destroyed"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass

    camera_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
