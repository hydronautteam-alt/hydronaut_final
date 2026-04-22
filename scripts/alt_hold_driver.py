#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State, OverrideRCIn
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, MessageInterval

class HydronautOrca4Logic(Node):
    def __init__(self):
        super().__init__('hydronaut_orca4_logic')
        
        prefix = "/mavros"
        
        # State Variables
        self.state = State()
        self.current_alt = 0.0
        self.have_pose = False
        
        # Subscriptions
        self.create_subscription(State, f"{prefix}/state", self.state_cb, 10)
        self.create_subscription(PoseStamped, f"{prefix}/local_position/pose", self.pose_cb, qos_profile_sensor_data)
        
        # Publishers
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.setpoint_pub = self.create_publisher(GeoPoseStamped, f"{prefix}/setpoint_position/global", best_effort_qos)
        self.rc_pub = self.create_publisher(OverrideRCIn, f"{prefix}/rc/override", 10)
        
        # Clients
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode")
        self.msg_interval_client = self.create_client(MessageInterval, f"{prefix}/set_message_interval")

        self.target_depth = -2.0
        self.phase = "init"
        self.global_timer = 0    # Tracks total mission time
        self.phase_timer = 0     # Tracks time spent in the current movement phase
        
        # 20Hz Control Loop (1 second = 20 ticks)
        self.timer = self.create_timer(0.05, self.mission_loop)
        
        self.get_logger().info("Hydronaut Controller Started. Waiting for connection...")

    def state_cb(self, msg): 
        self.state = msg

    def pose_cb(self, msg):
        self.current_alt = msg.pose.position.z
        if not self.have_pose:
            self.have_pose = True
            self.get_logger().info("EKF is running. Pose received.")

    def transition_to(self, new_phase):
        """Helper to switch phases and reset the phase timer."""
        self.phase = new_phase
        self.phase_timer = 0
        self.get_logger().info(f"--- STARTING MANEUVER: {new_phase.upper()} ---")

    def mission_loop(self):
        if not self.state.connected: 
            return

        # ---------------------------------------------------------
        # PRE-FLIGHT SEQUENCE
        # ---------------------------------------------------------
        if self.global_timer == 10:
            self.set_message_rate(31, 20.0) 
            self.set_message_rate(32, 20.0) 

        elif self.global_timer == 40:
            if self.have_pose:
                self.arm_vehicle(True)
            else:
                self.global_timer -= 1 

        elif self.global_timer == 80:
            self.set_mode("ALT_HOLD")
            self.transition_to("diving")

        # ---------------------------------------------------------
        # CHOREOGRAPHY SEQUENCE (Begins after tick 100)
        # ---------------------------------------------------------
        elif self.global_timer > 100:
            
            # Keep broadcasting the target depth to ArduSub
            self.publish_setpoint()

            # 1. DIVE TO DEPTH
            if self.phase == "diving":
                # Check if we are within 15cm of target
                if self.current_alt <= self.target_depth + 0.15:
                    self.transition_to("forward")

            # 2. MOVE FORWARD (approx 2 meters)
            elif self.phase == "forward":
                self.publish_rc(forward=1600)
                if self.phase_timer > 100: # 5 seconds
                    self.transition_to("yaw_cw")

            # 3. YAW CLOCKWISE (Right)
            elif self.phase == "yaw_cw":
                self.publish_rc(yaw=1600)
                if self.phase_timer > 60: # 3 seconds
                    self.transition_to("yaw_ccw")

            # 4. YAW COUNTER-CLOCKWISE (Left)
            elif self.phase == "yaw_ccw":
                self.publish_rc(yaw=1400)
                if self.phase_timer > 60: # 3 seconds
                    self.transition_to("strafe_left")

            # 5. STRAFE LEFT
            elif self.phase == "strafe_left":
                self.publish_rc(lateral=1400)
                if self.phase_timer > 60: # 3 seconds
                    self.transition_to("strafe_right")

            # 6. STRAFE RIGHT
            elif self.phase == "strafe_right":
                self.publish_rc(lateral=1600)
                if self.phase_timer > 60: # 3 seconds
                    self.transition_to("stop")

            # 7. MISSION COMPLETE (Hover in place)
            elif self.phase == "stop":
                self.publish_rc() # Defaults to 1500 (Stop)
                if self.phase_timer % 40 == 0:
                    self.get_logger().info("Mission Complete. Holding Position.")

            # Increment the timer for the current phase
            self.phase_timer += 1

        self.global_timer += 1

    # ==========================================
    # PUBLISHERS & SERVICE CALLS
    # ==========================================
    def publish_setpoint(self):
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.altitude = float(self.target_depth)
        self.setpoint_pub.publish(msg)

    def publish_rc(self, yaw=65535, forward=65535, lateral=65535):
        """
        Sends virtual joystick commands. 
        65535 tells ArduSub to ignore the channel and use auto-leveling.
        1500 is neutral/stop. 
        """
        msg = OverrideRCIn()
        msg.channels = [65535] * 18 
        
        # We only override the horizontal movement channels.
        # If a channel isn't explicitly passed to this function, it defaults to 1500 (Stop).
        msg.channels[3] = yaw if yaw != 65535 else 1500      # Channel 4
        msg.channels[4] = forward if forward != 65535 else 1500  # Channel 5
        msg.channels[5] = lateral if lateral != 65535 else 1500  # Channel 6
        
        self.rc_pub.publish(msg)

    def set_message_rate(self, msg_id, rate):
        req = MessageInterval.Request()
        req.message_id = msg_id
        req.message_rate = rate
        self.msg_interval_client.call_async(req)

    def set_mode(self, mode):
        req = SetMode.Request(custom_mode=mode)
        self.mode_client.call_async(req)

    def arm_vehicle(self, value):
        req = CommandBool.Request(value=value)
        self.arm_client.call_async(req)

def main():
    rclpy.init()
    node = HydronautOrca4Logic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY SHUTDOWN: Cut all horizontal motors before exiting!
        node.get_logger().info("Shutting down... Stopping motors.")
        node.publish_rc(yaw=1500, forward=1500, lateral=1500)
        rclpy.shutdown()

if __name__ == '__main__': 
    main()