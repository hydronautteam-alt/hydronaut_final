#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

msg = """
HYDRONAUT TELEOP CONTROL (10Hz CONTINUOUS)
----------------------------------------
HOLD keys to move. Release to stop.
w/s : Forward / Backward
a/d : Turn Left / Right
i/k : Heave UP / DOWN (Vertical)

r/v : Increase/Decrease Speed
CTRL-C to quit
"""

moveBindings = {
    'w': (0, 0, 1),   # Forward
    's': (0, 0, -1),  # Backward
    'a': (0, -1, 0),  # Turn Left
    'd': (0, 1, 0),   # Turn Right
    'i': (1, 0, 0),   # UP (Heave)
    'k': (-1, 0, 0),  # DOWN (Heave)
}

speedBindings = {
    'r': 0.1,
    'v': -0.1,
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # 0.1 second timeout makes this loop run at exactly 10Hz!
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_keyboard')
    
    # PUBLISH DIRECTLY TO MAVROS
    pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
    
    speed = 0.5
    settings = termios.tcgetattr(sys.stdin)

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            heave = 0.0
            yaw = 0.0
            throttle = 0.0

            if key in moveBindings.keys():
                heave = moveBindings[key][0]
                yaw = moveBindings[key][1]
                throttle = moveBindings[key][2]
            elif key in speedBindings.keys():
                speed += speedBindings[key]
                speed = max(0.0, min(1.0, speed))
                print(f"Speed: {int(speed*100)}%")
            elif key == '\x03': # CTRL-C
                break

            # ARDUSUB PWM MATH (1500 = Neutral, 1100 = Min, 1900 = Max)
            cmd = OverrideRCIn()
            cmd.channels = [65535] * 18  # 65535 means "ignore"
            
            # Keep core channels neutral to satisfy failsafe checks
            cmd.channels[0] = 1500  # Pitch
            cmd.channels[1] = 1500  # Roll
            cmd.channels[5] = 1500  # Lateral
            
            # Apply your keyboard commands!
            cmd.channels[2] = int(1500 + (heave * speed * 400))    # Ch 3: Heave
            cmd.channels[3] = int(1500 + (yaw * speed * 400))      # Ch 4: Yaw
            cmd.channels[4] = int(1500 + (throttle * speed * 400)) # Ch 5: Forward

            # Publish continuously at 10Hz!
            pub.publish(cmd)

    except Exception as e:
        print(e)

    finally:
        cmd = OverrideRCIn()
        cmd.channels = [65535] * 18
        cmd.channels[0:6] = [1500, 1500, 1500, 1500, 1500, 1500]
        pub.publish(cmd)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
