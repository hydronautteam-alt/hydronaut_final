#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from hydronaut_msgs.msg import Effort
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# --- ROS 2 Specific Imports for fixes ---
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# --- TUNING PARAMETERS ---
MAX_THRUST = 0.7
MAX_TORQUE = 0.8
SEARCH_SPEED = 0.5
TARGET_DEPTH = -1.5
TARGET_AREA = 200000.0
AREA_DEADZONE = 3000.0
CENTER_TOLERANCE = 40.0
BALL_CLASS_ID = 0
CONF_THRESHOLD = 0.15  # Increased from 0.15 to reduce false positives in pool reflections
FORWARD_GAIN = 0.000005
BRAKE_MULTIPLIER = 2.0
TURN_GAIN = 0.003
D_GAIN = 0.008

class AUVBallTrackerNode(Node):
    def __init__(self):
        super().__init__('auv_ball_tracker_node')

        self.effort_pub = self.create_publisher(Effort, '/hydronaut/cmd_effort', 10)
        self.depth_pub = self.create_publisher(Float32, '/hydronaut/cmd_depth', 10)

        self.system_active = True # Defaults to True so it runs immediately
        self.state_subscription = self.create_subscription(
            Bool,
            '/hydronaut/system_active',
            self.state_callback,
            10)
        # 1. FIXED QoS: Force RELIABLE connection to match the camera
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. FIXED Threading: Create a Reentrant group so inference doesn't block frames
        self.group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            reliable_qos,
            callback_group=self.group
        )

        self.bridge = CvBridge()
        self.last_error_x = 0.0
        self.frame_count = 0  # Counter for frame skipping

        package_share_directory = get_package_share_directory('hydronaut_pilot')
        model_path = os.path.join(package_share_directory, 'models', 'yolo11n_ncnn_model')

        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info("AI Tracker Online. QoS: RELIABLE. Listening for frames...")
        except Exception as e:
            self.get_logger().error(f"Model Load Failed: {e}")
            raise e

    def state_callback(self, msg):
        # Only log if the state actually changes
        if self.system_active != msg.data:
            self.system_active = msg.data
            state_str = "RESUMING" if self.system_active else "HALTING"
            self.get_logger().info(f"YOLO Tracking {state_str} by Master Command.")

    def image_callback(self, msg):

        if not self.system_active:
            return  # Skips the YOLO inference entirely. Saves Pi 5 CPU.
        
        # 3. FIXED Throttling: Skip every other frame to save CPU on the Pi 5
        self.frame_count += 1
        if self.frame_count % 2 != 0:
            return

        depth_msg = Float32()
        depth_msg.data = TARGET_DEPTH
        self.depth_pub.publish(depth_msg)

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2.0

        results = self.model.track(frame, persist=True, tracker="bytetrack.yaml",
                                   verbose=False, classes=[BALL_CLASS_ID], conf=CONF_THRESHOLD)

        effort_msg = Effort()
        target_found = False

        if results and len(results[0].boxes) > 0:
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
                error_x = float(box_center_x - frame_center_x)
                derivative = error_x - self.last_error_x
                self.last_error_x = error_x

                if abs(error_x) < CENTER_TOLERANCE:
                    raw_torque = 0.0
                else:
                    raw_torque = (error_x * TURN_GAIN) + (derivative * D_GAIN)

                error_area = TARGET_AREA - largest_area
                if abs(error_area) < AREA_DEADZONE:
                    base_thrust = 0.0
                    raw_torque *= 0.5
                else:
                    base_thrust = error_area * FORWARD_GAIN
                    if base_thrust < 0:
                        base_thrust *= BRAKE_MULTIPLIER

                speed_factor = max(0.0, 1.0 - (abs(error_x) / (frame_width / 2.5)))
                effort_msg.force.x = float(max(min(base_thrust * speed_factor, MAX_THRUST), -MAX_THRUST))
                effort_msg.torque.z = float(max(min(raw_torque, MAX_TORQUE), -MAX_TORQUE))

        if not target_found:
            effort_msg.force.x = 0.0
            effort_msg.torque.z = SEARCH_SPEED if self.last_error_x > 0 else -SEARCH_SPEED

        self.effort_pub.publish(effort_msg)



def main(args=None):
    rclpy.init(args=args)
    node = AUVBallTrackerNode()

    # 4. FIXED Executor: Use MultiThreaded so listening and inferencing happen simultaneously
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("User Interrupted. Stopping...")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # Clean up the OpenCV window on exit
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()