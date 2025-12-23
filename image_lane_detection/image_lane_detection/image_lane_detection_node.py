#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os

class ImageLaneDetectionNode(Node):
    def __init__(self):
        super().__init__("image_lane_detection_node")
        
        # Publisher for processed lane image
        self.publisher_ = self.create_publisher(Image, 'detected_lane', 10)

        # Parameters to choose input source
        self.declare_parameter('use_dataset', True)  # True: read from video file, False: subscribe to image topic
        self.declare_parameter('image_topic', 'camera/image_raw')
        # Name of the video file inside the package's resources folder
        self.declare_parameter('dataset_file', 'dataset.mp4')

        self.use_dataset = self.get_parameter('use_dataset').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.dataset_file = self.get_parameter('dataset_file').get_parameter_value().string_value

        # Declare and get debug parameter
        self.declare_parameter('debug', True)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        if self.debug:
            self.get_logger().info("Debug mode enabled. Publishing additional debug image topics.")
            # Debug publishers
            self.debug_canny_pub = self.create_publisher(Image, 'debug/canny_image', 10)
            self.debug_roi_pub = self.create_publisher(Image, 'debug/roi_image', 10)
            self.debug_line_pub = self.create_publisher(Image, 'debug/line_image', 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()

        if self.use_dataset:
            # Dataset videos are always under the package's resources folder.
            package_share_directory = get_package_share_directory('image_lane_detection')
            video_path = os.path.join(package_share_directory, 'resources', self.dataset_file)

            # Ensure the dataset file exists
            if not os.path.exists(video_path):
                raise FileNotFoundError(f"Dataset video file does not exist: {video_path}")
            
            # Open video file
            self.cap = cv2.VideoCapture(video_path)
            
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open video file: {video_path}")
            
            # Get video properties
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"Using dataset video. FPS: {self.fps}, Resolution: {self.width}x{self.height}")
            
            # Create timer to publish frames at video's frame rate
            timer_period = 1.0 / self.fps if self.fps > 0 else 0.033  # Default to ~30 FPS if unable to get FPS
            self.timer = self.create_timer(timer_period, self.timer_callback)
        else:
            # Subscribe to external image topic
            self.subscription = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f"Subscribed to image topic '{self.image_topic}' for lane detection.")
        
        self.get_logger().info("Image Lane Detection Node initialized.")
    
    def canny(self, image):
        """Convert image to grayscale, apply Gaussian blur, and Canny edge detection"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny
    
    def ROI(self, image):
        """Create a region of interest mask (trapezoidal region) - standard for lane detection"""
        height = image.shape[0]
        width = image.shape[1]
        # Define trapezoidal ROI - wider at bottom, narrower at top
        # This is more standard for lane detection than triangular
        polygons = np.array([[
            (int(width * 0.1), height),                    # Bottom left
            (int(width * 0.45), int(height * 0.6)),        # Top left
            (int(width * 0.55), int(height * 0.6)),        # Top right
            (int(width * 0.9), height)                     # Bottom right
        ]])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image
    
    def make_coordinates(self, image, line_parameters):
        """Calculate line coordinates from slope and intercept with bounds checking"""
        slope, intercept = line_parameters
        y1 = image.shape[0]  # Bottom of image
        y2 = int(y1 * 0.6)   # Top of ROI
        
        # Avoid division by zero
        if abs(slope) < 1e-6:
            return None
            
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        
        return np.array([x1, y1, x2, y2])
    
    def average_slope_intercept(self, image, lines):
        """Average the slopes and intercepts of detected lines to find left and right lane lines"""
        left_fit = []
        right_fit = []
        
        if lines is None:
            return None
        
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # Fit a line through the points
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            
            # Filter out near-horizontal lines (likely not lane lines)
            if abs(slope) < 0.5:
                continue
            
            # Left lane has negative slope, right lane has positive slope
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        
        # Need both lanes detected
        if len(left_fit) == 0 or len(right_fit) == 0:
            return None
        
        # Average the slopes and intercepts
        left_fit_avg = np.average(left_fit, axis=0)
        right_fit_avg = np.average(right_fit, axis=0)
        
        # Get coordinates for the averaged lines
        left_line = self.make_coordinates(image, left_fit_avg)
        right_line = self.make_coordinates(image, right_fit_avg)
        
        if left_line is None or right_line is None:
            return None
        
        return np.array([left_line, right_line])
    
    def display_lines(self, image, lines):
        """Draw lines on a blank image and optionally fill the lane area"""
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                # Draw the lane lines in green
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
            
            # Optionally fill the lane area (polygon between left and right lines)
            if len(lines) == 2:
                left_line = lines[0]
                right_line = lines[1]
                x1_l, y1_l, x2_l, y2_l = left_line.reshape(4)
                x1_r, y1_r, x2_r, y2_r = right_line.reshape(4)
                
                # Create polygon points
                lane_polygon = np.array([[
                    (x1_l, y1_l),
                    (x2_l, y2_l),
                    (x2_r, y2_r),
                    (x1_r, y1_r)
                ]], np.int32)
                
                # Fill the lane area with semi-transparent blue
                cv2.fillPoly(line_image, lane_polygon, (255, 0, 0))
        return line_image

    def process_frame(self, frame):
        """Run lane detection on a BGR image and publish results (and debug images)."""
        # Apply lane detection pipeline
        canny_image = self.canny(frame)
        cropped_image = self.ROI(canny_image)
        
        # Hough Line Transform with improved parameters
        # rho: distance resolution in pixels
        # theta: angular resolution in radians
        # threshold: minimum votes for a line to be detected
        # minLineLength: minimum line length
        # maxLineGap: maximum gap between line segments
        lines = cv2.HoughLinesP(
            cropped_image, 
            rho=2, 
            theta=np.pi/180, 
            threshold=100, 
            lines=np.array([]), 
            minLineLength=40, 
            maxLineGap=50
        )
        
        # Average the detected lines to get left and right lane lines
        averaged_lines = self.average_slope_intercept(frame, lines)
        
        if averaged_lines is not None:
            line_image = self.display_lines(frame, averaged_lines)
            # Blend the original image with the detected lane lines
            final_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        else:
            # If no lines detected, just use the original frame
            final_image = frame
        
        # Convert OpenCV image to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(final_image, encoding="bgr8")
        
        # Publish the image
        self.publisher_.publish(image_msg)

        # Publish debug images if debug mode is enabled
        if self.debug:
            # Canny and ROI images are single-channel; use mono8 encoding
            canny_msg = self.bridge.cv2_to_imgmsg(canny_image, encoding="mono8")
            roi_msg = self.bridge.cv2_to_imgmsg(cropped_image, encoding="mono8")
            self.debug_canny_pub.publish(canny_msg)
            self.debug_roi_pub.publish(roi_msg)

            if averaged_lines is not None:
                debug_line_image = self.display_lines(frame, averaged_lines)
                debug_line_msg = self.bridge.cv2_to_imgmsg(debug_line_image, encoding="bgr8")
                self.debug_line_pub.publish(debug_line_msg)

    def timer_callback(self):
        """Callback when using dataset video as input."""
        ret, frame = self.cap.read()
        
        if ret:
            self.process_frame(frame)
        else:
            # Video ended, loop back to the beginning
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info("Video ended. Looping back to start.")

    def image_callback(self, msg: Image):
        """Callback when subscribing to an image topic."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert incoming image: {e}")
            return

        self.process_frame(frame)
    
    def destroy_node(self):
        # Release video capture
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_lane_detection_node = ImageLaneDetectionNode()
    
    try:
        rclpy.spin(image_lane_detection_node)
    except KeyboardInterrupt:
        pass
    
    image_lane_detection_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
