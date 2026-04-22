import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class ArucoControlNode(Node):
    def __init__(self):
        super().__init__('aruco_master_control')
        self.bridge = CvBridge()
        
        # The Master Switch Flag
        self.system_active = True

        # Setup ArUco Detector (Jazzy / OpenCV 4.7+ API)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Camera Subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publisher to tell other nodes (Autonomy, Motors) if they should run
        self.state_publisher = self.create_publisher(Bool, '/hydronaut/system_active', 10)

        self.get_logger().info("ArUco Master Control Online. System Active.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 1. ALWAYS run ArUco detection (Takes < 1ms, very safe for CPU)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            if ids is not None:
                # --- THE STOP COMMAND (ID 7) ---
                if 7 in ids and self.system_active:
                    self.get_logger().warn("STOP MARKER (7) DETECTED. Halting operations.")
                    self.system_active = False
                    self.lock_thrusters()
                    self.publish_state()

                # --- THE START COMMAND (ID 10) ---
                elif 10 in ids and not self.system_active:
                    self.get_logger().info("START MARKER (10) DETECTED. Resuming operations.")
                    self.system_active = True
                    self.publish_state()

            # 2. THE GATEKEEPER LOGIC
            # If the system is stopped, exit the function immediately.
            if not self.system_active:
                return 

            # 3. HEAVY PROCESSING
            # --- Put your line tracking / person detection code here ---
            # This area will only execute when system_active is True.
            
        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")

    def lock_thrusters(self):
        # Publish 0 velocities to your motor controllers here
        # This ensures the bot doesn't drift away when it stops looking
        self.get_logger().info("Thrusters locked at 0 RPM.")

    def publish_state(self):
        # Broadcast the current state to the rest of the workspace
        state_msg = Bool()
        state_msg.data = self.system_active
        self.state_publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()