#!/usr/bin/env python3
"""
Simple Keyboard Teleop

Publishes to /cmd_vel_raw (which gets smoothed by velocity_smoother)

Controls:
    W/↑ : Forward
    S/↓ : Backward  
    A/← : Turn Left
    D/→ : Turn Right
    Q   : Forward + Left
    E   : Forward + Right
    SPACE : Stop (zero velocity)
    ESC/Ctrl+C : Quit

Velocity is set directly (step input), smoothed by velocity_smoother node.
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# For keyboard input
import termios
import tty
import select


# Key codes
ARROW_PREFIX = '\x1b'
ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Velocity settings (these are targets, smoother will ramp to them)
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 0.2   # rad/s
        
        # Current command
        self.linear = 0.0
        self.angular = 0.0
        
        # Publisher (to raw topic, smoother will process)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Timer to publish at steady rate
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz
        
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("Keyboard Teleop (with Velocity Smoother)")
        print("="*50)
        print(f"Linear speed:  {self.linear_speed} m/s")
        print(f"Angular speed: {self.angular_speed} rad/s")
        print("-"*50)
        print("Controls:")
        print("  W / ↑     : Forward")
        print("  S / ↓     : Backward")
        print("  A / ←     : Turn Left")
        print("  D / →     : Turn Right")
        print("  Q         : Forward + Turn Left")
        print("  E         : Forward + Turn Right")
        print("  SPACE     : Stop")
        print("  +/-       : Increase/Decrease linear speed")
        print("  [/]       : Increase/Decrease angular speed")
        print("  ESC/Ctrl+C: Quit")
        print("="*50 + "\n")

    def get_key(self, timeout=0.1):
        """Get a single keypress (non-blocking)"""
        if not sys.stdin.isatty():
            return None
            
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
                # Check for arrow key (escape sequence)
                if key == '\x1b':
                    # Read the rest of the escape sequence
                    if select.select([sys.stdin], [], [], 0.01)[0]:
                        key += sys.stdin.read(2)
                return key
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def publish_cmd(self):
        """Publish current velocity command"""
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    def run(self):
        """Main loop: read keys and update velocity"""
        print("Ready! Press keys to move...")
        
        try:
            while rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)
                
                # Get keypress
                key = self.get_key(0.05)
                
                if key is None:
                    continue
                
                # Process key
                if key in ['w', 'W', ARROW_UP]:
                    self.linear = self.linear_speed
                    self.angular = 0.0
                    print(f"\rForward  (v={self.linear:.2f})      ", end='', flush=True)
                    
                elif key in ['s', 'S', ARROW_DOWN]:
                    self.linear = -self.linear_speed
                    self.angular = 0.0
                    print(f"\rBackward (v={self.linear:.2f})      ", end='', flush=True)
                    
                elif key in ['a', 'A', ARROW_LEFT]:
                    self.linear = 0.0
                    self.angular = self.angular_speed
                    print(f"\rLeft     (ω={self.angular:.2f})      ", end='', flush=True)
                    
                elif key in ['d', 'D', ARROW_RIGHT]:
                    self.linear = 0.0
                    self.angular = -self.angular_speed
                    print(f"\rRight    (ω={self.angular:.2f})      ", end='', flush=True)
                    
                elif key in ['q', 'Q']:
                    self.linear = self.linear_speed
                    self.angular = self.angular_speed
                    print(f"\rFwd+Left (v={self.linear:.2f}, ω={self.angular:.2f})", end='', flush=True)
                    
                elif key in ['e', 'E']:
                    self.linear = self.linear_speed
                    self.angular = -self.angular_speed
                    print(f"\rFwd+Right (v={self.linear:.2f}, ω={self.angular:.2f})", end='', flush=True)
                    
                elif key == ' ':
                    self.linear = 0.0
                    self.angular = 0.0
                    print("\rStop                    ", end='', flush=True)
                    
                elif key == '+' or key == '=':
                    self.linear_speed = min(self.linear_speed + 0.1, 2.0)
                    print(f"\rLinear speed: {self.linear_speed:.2f} m/s    ", end='', flush=True)
                    
                elif key == '-' or key == '_':
                    self.linear_speed = max(self.linear_speed - 0.1, 0.1)
                    print(f"\rLinear speed: {self.linear_speed:.2f} m/s    ", end='', flush=True)
                    
                elif key == ']':
                    self.angular_speed = min(self.angular_speed + 0.1, 3.0)
                    print(f"\rAngular speed: {self.angular_speed:.2f} rad/s  ", end='', flush=True)
                    
                elif key == '[':
                    self.angular_speed = max(self.angular_speed - 0.1, 0.1)
                    print(f"\rAngular speed: {self.angular_speed:.2f} rad/s  ", end='', flush=True)
                    
                elif key == '\x1b' and len(key) == 1:  # ESC (not arrow)
                    print("\n\nExiting...")
                    break
                    
                elif key == '\x03':  # Ctrl+C
                    print("\n\nExiting...")
                    break
                    
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Send stop command
            self.linear = 0.0
            self.angular = 0.0
            self.publish_cmd()
            print("Stopped.")


def main():
    rclpy.init()
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure stop command is sent
        msg = Twist()
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()