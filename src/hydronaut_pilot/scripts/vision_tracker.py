#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from hydronaut_msgs.msg import TargetTracking

class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_tracker')
        
        # Change this to match your Gazebo camera topic if needed!
        self.camera_topic = '/camera/image_raw'
        self.bridge = CvBridge()
        
        # Subscriber to Gazebo Camera
        self.create_subscription(Image, self.camera_topic, self.image_cb, 10)
        
        # Publisher for the Brain
        self.target_pub = self.create_publisher(TargetTracking, '/hydronaut/target_tracking', 10)
        
        self.get_logger().info("Vision Tracker Started. Looking for Red...")

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        height, width, _ = cv_image.shape
        center_x_frame = width / 2
        center_y_frame = height / 2

        # Convert to HSV color space for reliable color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # RED wraps around the HSV cylinder, requiring two ranges
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        full_mask = mask1 + mask2

        contours, _ = cv2.findContours(full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        track_msg = TargetTracking()
        track_msg.header.stamp = self.get_clock().now().to_msg()
        track_msg.is_tracking = False

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Only track if the object is large enough (filters noise)
            if area > 500:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Convert pixel coordinates to [-1.0 to 1.0] scale
                    # X: Left = -1.0, Right = 1.0
                    track_msg.x_offset = (cx - center_x_frame) / center_x_frame
                    # Y: Top = -1.0, Bottom = 1.0
                    track_msg.y_offset = (cy - center_y_frame) / center_y_frame
                    
                    track_msg.is_tracking = True
                    
                    # Draw a green circle on the target for debugging
                    cv2.circle(cv_image, (cx, cy), 15, (0, 255, 0), -1)

        self.target_pub.publish(track_msg)

        # Show the camera feed (Helpful for debugging, you can comment this out later)
        cv2.imshow("Hydronaut Vision", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = VisionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()