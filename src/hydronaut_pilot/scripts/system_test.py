#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from std_msgs.msg import Float32
from hydronaut_msgs.msg import Effort

class HydronautSystemTest(Node):
    def __init__(self):
        super().__init__('system_test')
        
        # Publishers
        self.effort_pub = self.create_publisher(Effort, '/hydronaut/cmd_effort', 10)
        self.depth_cmd_pub = self.create_publisher(Float32, '/hydronaut/cmd_depth', 10)
        
        # Subscriber to wait for the Manager
        self.create_subscription(State, '/mavros/state', self.mavros_state_cb, 10)

        # Sequence state
        self.is_ready = False
        self.tick = 0
        
        # We run the logic loop at 10Hz (10 ticks per second)
        self.timer = self.create_timer(0.1, self.test_sequence)
        
        self.get_logger().info("System Test Node Initialized. Waiting for Auto-Boot...")

    def mavros_state_cb(self, msg):
        if msg.armed and msg.mode == "ALT_HOLD" and not self.is_ready:
            self.get_logger().info("Submarine is ARMED. Commencing System Test Dance in 3 seconds!")
            self.is_ready = True

    def publish_effort(self, x=0.0, y=0.0, torque_z=0.0):
        msg = Effort()
        msg.force.x = float(x)
        msg.force.y = float(y)
        msg.torque.z = float(torque_z)
        self.effort_pub.publish(msg)

    def test_sequence(self):
        if not self.is_ready:
            return

        self.tick += 1

        # --- Phase 1: Dive & Stabilize (0 to 50 ticks = 5 seconds) ---
        if self.tick == 1:
            self.get_logger().info("PHASE 1: Diving to -1.5m")
            depth_msg = Float32()
            depth_msg.data = -1.5
            self.depth_cmd_pub.publish(depth_msg)
            self.publish_effort(0.0, 0.0, 0.0)

        # --- Phase 2: Forward (50 to 80 ticks = 3 seconds) ---
        elif self.tick == 50:
            self.get_logger().info("PHASE 2: Surging Forward (X = 0.5)")
            self.publish_effort(x=0.5)

        # --- Phase 3: Backward (80 to 110 ticks = 3 seconds) ---
        elif self.tick == 80:
            self.get_logger().info("PHASE 3: Surging Backward (X = -0.5)")
            self.publish_effort(x=-0.5)

        # --- Phase 4: Strafe Right (110 to 140 ticks = 3 seconds) ---
        elif self.tick == 110:
            self.get_logger().info("PHASE 4: Strafing Right (Y = 0.5)")
            self.publish_effort(y=0.5)

        # --- Phase 5: Strafe Left (140 to 170 ticks = 3 seconds) ---
        elif self.tick == 140:
            self.get_logger().info("PHASE 5: Strafing Left (Y = -0.5)")
            self.publish_effort(y=-0.5)

        # --- Phase 6: Yaw Clockwise (170 to 200 ticks = 3 seconds) ---
        elif self.tick == 170:
            self.get_logger().info("PHASE 6: Yawing Clockwise (Torque Z = 0.5)")
            self.publish_effort(torque_z=0.5)

        # --- Phase 7: Yaw Counter-Clockwise (200 to 230 ticks = 3 seconds) ---
        elif self.tick == 200:
            self.get_logger().info("PHASE 7: Yawing Counter-Clockwise (Torque Z = -0.5)")
            self.publish_effort(torque_z=-0.5)

        # --- Phase 8: Full Stop (230 ticks) ---
        elif self.tick == 230:
            self.get_logger().info("SYSTEM TEST COMPLETE. Hovering at depth.")
            self.publish_effort(0.0, 0.0, 0.0)
            
        # Optional: Auto-shutdown the node after the test is done
        elif self.tick == 260:
            self.get_logger().info("Test script shutting down gracefully.")
            raise SystemExit

def main():
    rclpy.init()
    node = HydronautSystemTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()