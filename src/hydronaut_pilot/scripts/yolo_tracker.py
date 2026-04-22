#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from hydronaut_msgs.msg import Effort
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data

# ==========================================
# --- ANTI-WOBBLE TUNING PARAMETERS ---
# ==========================================
MAX_THRUST = 0.7         # Capped at 50% to prevent runaway speed
MAX_TORQUE = 0.8         # Torque for turning
SEARCH_SPEED = 0.5      # Spin speed when ball is lost
TARGET_DEPTH = -1.5      

# YOLO/Distance Tuning
TARGET_AREA = 200000.0    # Desired ball size (Pixels^2). Increase to stop further away.
AREA_DEADZONE = 3000.0   # Forward/Back safe zone. Stops motors when close enough.
CENTER_TOLERANCE = 40.0  # Left/Right safe zone (Pixels). Stops turning if within this range.
BALL_CLASS_ID = 0       # COCO 'sports ball'
CONF_THRESHOLD = 0.15    

# PID / Motion Logic
FORWARD_GAIN = 0.000005   # VERY LOW gain for smooth acceleration
BRAKE_MULTIPLIER = 2.0   # 200% power when moving REVERSE to fight momentum
TURN_GAIN = 0.003        # P-Gain for horizontal centering (Lower = softer turns)
D_GAIN = 0.008           # D-Gain (Higher = kills wobble/overshoot)

class AUVBallTrackerNode(Node):
    def __init__(self):
        super().__init__('auv_ball_tracker_node')
        
        # --- AUV Publishers ---
        self.effort_pub = self.create_publisher(Effort, '/hydronaut/cmd_effort', 10)
        self.depth_pub = self.create_publisher(Float32, '/hydronaut/cmd_depth', 10)
        
        # --- AUV Subscribers ---
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        ) 
        self.bridge = CvBridge()

        # State tracking for Derivative (D-Gain)
        self.last_error_x = 0.0 

        # --- Model Initialization ---
        package_share_directory = get_package_share_directory('hydronaut_pilot') 
        model_path = os.path.join(package_share_directory, 'models', 'yolo11n_ncnn_model')
        
        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info("Smooth Ball Tracker Active. Anti-Wobble Enabled.")
        except Exception as e:
            self.get_logger().error(f"Model Load Failed: {e}")
            raise e

    def image_callback(self, msg):
        # 1. Maintain Depth Command
        depth_msg = Float32()
        depth_msg.data = TARGET_DEPTH
        self.depth_pub.publish(depth_msg)

        # 2. Convert ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: return

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2.0

        # 3. YOLO Inference (Tracking mode)
        results = self.model.track(
            frame, persist=True, tracker="bytetrack.yaml", 
            verbose=False, classes=[BALL_CLASS_ID], conf=CONF_THRESHOLD
        )

        effort_msg = Effort()
        target_found = False

        if results and len(results[0].boxes) > 0:
            # Find the largest ball in the frame
            largest_area = 0
            target_box = None
            for box in results[0].boxes:
                coords = box.xyxy.cpu().numpy()[0]
                area = (coords[2] - coords[0]) * (coords[3] - coords[1])
                if area > largest_area:
                    largest_area = area
                    target_box = coords

            if target_box is not None:
                target_found = True
                box_center_x = (target_box[0] + target_box[2]) / 2.0
                
                # ==========================================
                # --- HORIZONTAL CENTERING (YAW) ---
                # ==========================================
                error_x = float(box_center_x - frame_center_x)
                derivative = error_x - self.last_error_x
                self.last_error_x = error_x
                
                # Center Deadzone: Ignore tiny left/right movements to stop wobble
                if abs(error_x) < CENTER_TOLERANCE:
                    raw_torque = 0.0
                else:
                    raw_torque = (error_x * TURN_GAIN) + (derivative * D_GAIN)

                # ==========================================
                # --- DISTANCE CONTROL (FORWARD/BACK) ---
                # ==========================================
                error_area = TARGET_AREA - largest_area
                
                # Distance Deadzone: If close enough, cut motors entirely
                if abs(error_area) < AREA_DEADZONE:
                    base_thrust = 0.0
                    raw_torque *= 0.5  # Also dampen any leftover turning
                else:
                    base_thrust = error_area * FORWARD_GAIN
                    
                    # BRAKING: Scale up reverse thrust to stop momentum
                    if base_thrust < 0:
                        base_thrust *= BRAKE_MULTIPLIER

                # --- MOTION SMOOTHING ---
                # Cut forward speed if turning sharply so it doesn't circle the ball
                speed_factor = max(0.0, 1.0 - (abs(error_x) / (frame_width / 2.5)))

                effort_msg.force.x = float(max(min(base_thrust * speed_factor, MAX_THRUST), -MAX_THRUST))
                effort_msg.torque.z = float(max(min(raw_torque, MAX_TORQUE), -MAX_TORQUE))

        # ==========================================
        # --- SEARCH BEHAVIOR ---
        # ==========================================
        if not target_found:
            effort_msg.force.x = 0.0
            # Spin toward the side where the ball was last seen
            effort_msg.torque.z = SEARCH_SPEED if self.last_error_x > 0 else -SEARCH_SPEED

        # 4. Final Command Dispatch
        self.effort_pub.publish(effort_msg)

        # 5. Visualization Window
        annotated_frame = results[0].plot()
        
        # Add debug text to screen
        status_text = "Tracking" if target_found else "Searching"
        cv2.putText(annotated_frame, f"State: {status_text}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if target_found else (0, 0, 255), 2)
        cv2.putText(annotated_frame, f"Thrust: {effort_msg.force.x:.2f}", (20, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(annotated_frame, f"Torque: {effort_msg.torque.z:.2f}", (20, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw the Center Safe Zone lines
        safe_left = int(frame_center_x - CENTER_TOLERANCE)
        safe_right = int(frame_center_x + CENTER_TOLERANCE)
        cv2.line(annotated_frame, (safe_left, 0), (safe_left, int(frame_height)), (255, 255, 0), 1)
        cv2.line(annotated_frame, (safe_right, 0), (safe_right, int(frame_height)), (255, 255, 0), 1)
        
        cv2.imshow("AUV Ball Tracker", annotated_frame)
        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = AUVBallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User Interrupted. Stopping...")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()