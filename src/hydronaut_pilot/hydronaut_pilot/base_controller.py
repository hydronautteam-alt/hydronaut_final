#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import OverrideRCIn
from hydronaut_msgs.msg import Effort

class HydronautBaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers and Subscribers
        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", reliable_qos)
        self.create_subscription(Effort, "/hydronaut/cmd_effort", self.effort_cb, 10)
        
        self.get_logger().info("Base Controller Online. Listening for Effort commands...")

    def effort_cb(self, msg: Effort):
        rc_msg = OverrideRCIn()
        
        # 65535 tells ArduSub to ignore the channel and maintain ALT_HOLD stability
        rc_msg.channels = [65535] * 18 
        
        # Channel 4 (Yaw) -> torque.z
        rc_msg.channels[3] = self.effort_to_pwm(msg.torque.z)
        # Channel 5 (Forward/Surge) -> force.x
        rc_msg.channels[4] = self.effort_to_pwm(msg.force.x)
        # Channel 6 (Lateral/Sway) -> force.y
        rc_msg.channels[5] = self.effort_to_pwm(msg.force.y)

        self.rc_pub.publish(rc_msg)

    def effort_to_pwm(self, effort_val):
        effort_val = max(-1.0, min(1.0, float(effort_val)))
        if effort_val == 0.0:
            return 65535 
        return int(1500 + (effort_val * 400))

def main():
    rclpy.init()
    node = HydronautBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down... Stopping horizontal motors.")
        stop_msg = OverrideRCIn()
        stop_msg.channels = [65535] * 18 
        for i in range(3, 6): stop_msg.channels[i] = 1500
        node.rc_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()