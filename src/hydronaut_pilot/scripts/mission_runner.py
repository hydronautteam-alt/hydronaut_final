#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from std_msgs.msg import Float32 
from hydronaut_msgs.msg import TargetTracking, Effort

class HydronautMissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')
        
        # Publishers
        self.effort_pub = self.create_publisher(Effort, '/hydronaut/cmd_effort', 10)
        self.depth_cmd_pub = self.create_publisher(Float32, '/hydronaut/cmd_depth', 10)
        
        # Subscribers
        self.create_subscription(TargetTracking, '/hydronaut/target_tracking', self.tracking_cb, 10)
        self.create_subscription(State, '/mavros/state', self.mavros_state_cb, 10)

        # --- PD YAW PARAMETERS ---
        self.kp_yaw = 0.6  
        self.kd_yaw = 0.2
        self.prev_x_error = 0.0
        
        # --- PROPORTIONAL DEPTH PARAMETERS ---
        self.target_depth = -1.0     # Initial search depth (1 meter underwater)
        self.kp_depth = 0.007        # Proportional gain (max 5mm change per frame)
        self.vertical_deadzone = 0.15 # 15% center zone where we don't adjust depth
        
        # Safety Limits (Don't fly into the air, don't crash into the floor)
        self.max_depth = -0.2 
        self.min_depth = -2.8 

        self.is_ready = False
        self.get_logger().info("Mission Runner waiting for Auto-Boot sequence...")

    def mavros_state_cb(self, msg):
        # Wait until Manager has safely armed and engaged ALT_HOLD
        if msg.armed and msg.mode == "ALT_HOLD" and not self.is_ready:
            self.get_logger().info("SUBMARINE IS ARMED AND READY. STARTING PURSUIT!")
            self.is_ready = True
            
            # Send initial depth command to the Manager
            self.publish_depth_command(self.target_depth)

    def publish_depth_command(self, depth: float):
        msg = Float32()
        msg.data = depth
        self.depth_cmd_pub.publish(msg)

    def tracking_cb(self, msg: TargetTracking):
        if not self.is_ready:
            return 

        effort_msg = Effort()
        
        if msg.is_tracking:
            # ==========================================
            # 1. PD YAW CONTROL (Left/Right Steering)
            # ==========================================
            current_x_error = msg.x_offset
            d_error = current_x_error - self.prev_x_error
            yaw_effort = (self.kp_yaw * current_x_error) + (self.kd_yaw * d_error)
            self.prev_x_error = current_x_error

            # ==========================================
            # 2. PROPORTIONAL DEPTH CONTROL (Up/Down)
            # ==========================================
            if abs(msg.y_offset) > self.vertical_deadzone:
                # Calculate the exact adjustment based on how far away the ball is
                depth_adjustment = msg.y_offset * self.kp_depth
                
                # Apply the adjustment to our target depth
                self.target_depth -= depth_adjustment
                
                # SAFETY CLAMP: Keep the target depth within safe pool limits
                self.target_depth = max(self.min_depth, min(self.max_depth, self.target_depth))
                
                self.publish_depth_command(self.target_depth)

            # ==========================================
            # 3. FORWARD MOTION
            # ==========================================
            effort_msg.force.x = 0.3      # Constant 30% forward thrust
            effort_msg.torque.z = yaw_effort 
            
        else:
            # ==========================================
            # SEARCH PATTERN (If ball is lost)
            # ==========================================
            effort_msg.force.x = 0.0
            effort_msg.torque.z = 0.15 # Spin slowly clockwise
            
        self.effort_pub.publish(effort_msg)

def main():
    rclpy.init()
    node = HydronautMissionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()