#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, SetMode, MessageInterval
from std_msgs.msg import Float32 

class HydronautManager(Node):
    def __init__(self):
        super().__init__('manager')
        
        prefix = "/mavros"
        self.mav_msg_ids = [31, 32] 
        self.mav_msg_rate = 20.0
        
        # State Variables
        self.ardusub_connected = False
        self.ardusub_armed = False
        self.ardusub_mode = ""
        self.have_pose = False
        self.target_depth = -2.0
        
        # QoS Profiles
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriptions
        self.create_subscription(State, f"{prefix}/state", self.state_cb, reliable_qos)
        self.create_subscription(PoseStamped, f"{prefix}/local_position/pose", self.pose_cb, best_effort_qos)
        self.create_subscription(Float32, "/hydronaut/cmd_depth", self.depth_cmd_cb, 10)
        
        # Publishers
        self.setpoint_pub = self.create_publisher(GeoPoseStamped, f"{prefix}/setpoint_position/global", best_effort_qos)
        
        # Service Clients
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode")
        self.msg_interval_client = self.create_client(MessageInterval, f"{prefix}/set_message_interval")

        # Timers
        self.mode_timer = None
        self.mav_msg_rate_timer = None
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint) 
        
        self.get_logger().info("Manager ready. Waiting for ArduSub connection...")

    def state_cb(self, msg: State): 
        if self.ardusub_connected != msg.connected:
            self.ardusub_connected = msg.connected
            if self.ardusub_connected:
                self.get_logger().info("ArduSub connected")
                self.mode_timer = self.create_timer(1.0, self.go_auv)
                self.mav_msg_rate_timer = self.create_timer(10.0, self.set_message_rates)
            else:
                self.get_logger().info("ArduSub disconnected")
                if self.mode_timer: self.mode_timer.cancel()
                if self.mav_msg_rate_timer: self.mav_msg_rate_timer.cancel()
                    
        if self.ardusub_armed != msg.armed:
            self.ardusub_armed = msg.armed
            self.get_logger().info("armed" if msg.armed else "disarmed")

        if self.ardusub_mode != msg.mode:
            self.ardusub_mode = msg.mode
            self.get_logger().info(f"ArduSub mode is {msg.mode}")

    def pose_cb(self, msg: PoseStamped):
        if not self.have_pose:
            self.have_pose = True
            self.get_logger().info("EKF is running")

    def depth_cmd_cb(self, msg: Float32):
        new_depth = -abs(msg.data)
        if new_depth != self.target_depth:
            self.target_depth = new_depth
            self.get_logger().info(f"New Target Depth: {self.target_depth:.2f}m")

    def set_arm(self, arm: bool):
        self.arm_client.call_async(CommandBool.Request(value=arm))

    def set_ardusub_mode(self, custom_mode: str):
        self.mode_client.call_async(SetMode.Request(custom_mode=custom_mode))

    def set_message_rates(self):
        for msg_id in self.mav_msg_ids:
            self.msg_interval_client.call_async(MessageInterval.Request(message_id=msg_id, message_rate=float(self.mav_msg_rate)))

    def go_auv(self):
        if self.ardusub_connected and self.have_pose:
            if not self.ardusub_armed:
                self.set_arm(True)
            if self.ardusub_mode != "ALT_HOLD":
                self.set_ardusub_mode("ALT_HOLD")

    def publish_setpoint(self):
        if self.ardusub_connected and self.have_pose:
            msg = GeoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.altitude = float(self.target_depth)
            self.setpoint_pub.publish(msg)

def main():
    rclpy.init()
    node = HydronautManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()