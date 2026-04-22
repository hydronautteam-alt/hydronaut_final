#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32 
from hydronaut_msgs.msg import Effort
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# --- TUNING PARAMETERS ---
MAX_SPEED = 0.4         
TURN_GAIN = 0.008       # INCREASED: Turn harder when the line is off-center
D_GAIN = 0.004          # INCREASED: More braking to prevent wobbling after a sharp turn
SEARCH_SPEED = 0.2      # Slightly faster search spin
TARGET_DEPTH = -2.0     

# COLOR SETTINGS (HSV) - BLUE
LOWER_COLOR = np.array([100, 50, 50])   
UPPER_COLOR = np.array([130, 255, 255]) 

class AUVLineTrackingNode(Node):
    def __init__(self):
        super().__init__('line_tracking_node')
        
        self.effort_pub = self.create_publisher(Effort, '/hydronaut/cmd_effort', 10)
        self.depth_pub = self.create_publisher(Float32, '/hydronaut/cmd_depth', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        
        self.bridge = CvBridge()
        self.last_error = 0.0 
        self.get_logger().info("AUV Line Tracking: Sharp Turn Update Initialized!")

    def listener_callback(self, data):
        depth_msg = Float32()
        depth_msg.data = TARGET_DEPTH
        self.depth_pub.publish(depth_msg)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            return

        height, width, _ = cv_image.shape
        crop_start_y = int(height * 1 / 2) # Start crop a bit higher to see further ahead
        crop_end_y = int(height)
        
        # FIX 1: REMOVED BLINDERS. We want the full width of the camera.
        cropped = cv_image[crop_start_y:crop_end_y, 0:width]
        crop_h, crop_w, _ = cropped.shape

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > 300]
        target_contour = None

        if valid_contours:
            # Still finding the left-most contour
            left_most_cx = 99999
            for c in valid_contours:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    if cx < left_most_cx:
                        left_most_cx = cx
                        target_contour = c

        effort_msg = Effort()

        if target_contour is not None:
            M = cv2.moments(target_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                error = float(cx - (crop_w // 2))
                derivative = error - self.last_error
                self.last_error = error
                
                # FIX 2: Aggressive speed reduction. 
                # If error is close to the edge (crop_w/2), speed_factor becomes 0.
                speed_factor = max(0.0, 1.0 - (abs(error) / (crop_w / 2.0)))
                
                # Drop to 0.0 forward speed on sharp turns so it can pivot.
                effort_msg.force.x = MAX_SPEED * speed_factor 
                
                # Steering command
                effort_msg.torque.z = (error * TURN_GAIN) + (derivative * D_GAIN)
                
                cv2.drawContours(cropped, [target_contour], -1, (0, 255, 0), 3)
                cv2.circle(cropped, (cx, cy), 5, (0, 0, 255), -1)
        else:
            effort_msg.force.x = 0.0
            if self.last_error > 0:
                effort_msg.torque.z = SEARCH_SPEED 
            elif self.last_error < 0:
                effort_msg.torque.z = -SEARCH_SPEED 
            else:
                effort_msg.torque.z = 0.0

        self.effort_pub.publish(effort_msg)
        cv2.imshow("AUV View (Color Mask)", mask) 
        cv2.imshow("AUV View (Overlay)", cropped)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AUVLineTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()