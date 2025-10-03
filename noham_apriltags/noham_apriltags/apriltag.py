#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

# NumPy compatibility fix for cv_bridge and cv2
import sys
import os

# Set NumPy compatibility environment variables before importing cv2/cv_bridge
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '0'
os.environ['NUMPY_EXPERIMENTAL_DTYPE_API'] = '0'

# Try importing with error handling
try:
    from cv_bridge import CvBridge
    import cv2
except (AttributeError, ImportError) as e:
    print(f"Initial import error: {e}")
    # Force reload numpy and try again
    import numpy as np
    print(f"NumPy version: {np.__version__}")
    
    # Try to patch missing attributes if they don't exist
    if not hasattr(np, '_ARRAY_API'):
        np._ARRAY_API = None
    
    try:
        # Retry imports after patching
        from cv_bridge import CvBridge
        import cv2
        print("Successfully imported cv_bridge and cv2 after patching")
    except Exception as e2:
        print(f"Failed to import even after patching: {e2}")
        # As a last resort, try downgrading numpy programmatically
        import subprocess
        print("Attempting to downgrade NumPy...")
        try:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'numpy<2'])
            print("NumPy downgraded successfully. Please restart the node.")
            sys.exit(0)
        except Exception as e3:
            print(f"Failed to downgrade NumPy: {e3}")
            raise e2

from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        camera_info_subs_ = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.info_listener_callback,
            10)
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.rgb_listener_callback,
            10)
        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)
        self.bridge = CvBridge()
        self.target_detected = False  # Fixed typo: was target_detectted
        self.target_center = None
        self.target_id = None
        self.tag_area = float
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detection = None
        self.image_area = None
        self.image_width = None
        self.image_height = None
        self.image_area = None
        self.get_logger().info("AprilTag Detector Node Initialized")

    def rgb_listener_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.target_detected and self.target_center and self.detection:
            # Draw a box around the AprilTag
            corners = [(int(corner.x), int(corner.y)) for corner in self.detection.corners]
            cv2.drawContours(current_frame, [np.array(corners)], 0, (0, 255, 0), 2)
            # Draw the tag ID
            tag_id_text = f"Tag ID: {self.target_id}"
            cv2.putText(current_frame, tag_id_text, (int(self.target_center.x), int(self.target_center.y) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

    def info_listener_callback(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def detection_callback(self, msg):
        if msg.detections:
            self.target_detected = True
            self.detection = msg.detections[0]
            self.target_center = self.detection.centre
            self.target_id = self.detection.id
            tag_width = abs(self.detection.corners[0].x - self.detection.corners[2].x)
            tag_height = abs(self.detection.corners[0].y - self.detection.corners[2].y)
            self.tag_area = tag_width * tag_height
            self.image_area = self.image_width * self.image_height
        else:
            self.target_detected = False
            self.detection = None
            self.target_center = None
            self.target_id = None
            self.tag_area = None
        self.get_logger().info(f"Detected Tag ID: {self.target_id}, Area: {self.tag_area}")
        self.get_logger().info(f"Target Detected: {self.target_detected}")
        self.get_logger().info(f"Image Area: {self.image_area}")
        self.get_logger().info(f"Tag Area: {self.tag_area}")
        self.get_logger().info(f"Target Center: {self.target_center}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()