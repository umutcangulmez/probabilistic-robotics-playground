#!/usr/bin/env python3
"""
Velocity Smoother Node

Sits between teleop and robot, applies acceleration limits to create
smooth velocity profiles instead of step changes.

Teleop (step) → Smoother (ramp) → Robot (smooth motion)

Features:
- Linear and angular acceleration limits
- Deceleration can be different from acceleration
- Publishes at high rate for smooth control
- Handles timeout (auto-stop if no commands)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np


class VelocitySmoother(Node):
    def __init__(self):
        super().__init__('velocity_smoother')
        
        # ================================================================
        # PARAMETERS (can be set via launch file)
        # ================================================================
        # Declare parameters with defaults
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_linear_decel', 1.0)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('max_angular_decel', 2.0)
        self.declare_parameter('max_linear_jerk', 2.0)
        self.declare_parameter('max_angular_jerk', 4.0)
        self.declare_parameter('use_jerk_limit', True)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')

        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_linear_decel = self.get_parameter('max_linear_decel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.max_linear_jerk = self.get_parameter('max_linear_jerk').value
        self.max_angular_jerk = self.get_parameter('max_angular_jerk').value
        self.use_jerk_limit = self.get_parameter('use_jerk_limit').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # ================================================================
        # STATE
        # ================================================================
        self.target_linear = 0.0    # Target from teleop
        self.target_angular = 0.0
        
        self.current_linear = 0.0   # Current smoothed output
        self.current_angular = 0.0
        
        self.current_linear_accel = 0.0   # For jerk limiting
        self.current_angular_accel = 0.0
        
        self.last_cmd_time = self.get_clock().now()
        
        # ================================================================
        # ROS
        # ================================================================
        # Subscribe to raw teleop commands
        self.sub = self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        
        # Publish smoothed commands
        self.pub = self.create_publisher(Twist, output_topic, 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f"Velocity Smoother started")
        self.get_logger().info(f"  Linear accel: {self.max_linear_accel} m/s²")
        self.get_logger().info(f"  Angular accel: {self.max_angular_accel} rad/s²")
        self.get_logger().info(f"  Subscribe: /cmd_vel_raw → Publish: /cmd_vel")

    def cmd_callback(self, msg: Twist):
        """Receive raw teleop command (step input)"""
        # Clamp to max velocities
        self.target_linear = np.clip(msg.linear.x, -self.max_linear, self.max_linear)
        self.target_angular = np.clip(msg.angular.z, -self.max_angular, self.max_angular)
        
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        """Publish smoothed velocity at fixed rate"""
        dt = 1.0 / self.publish_rate
        
        # Check for timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if time_since_cmd > self.cmd_timeout:
            # No recent command → target is zero (stop)
            self.target_linear = 0.0
            self.target_angular = 0.0
        
        # Smooth linear velocity
        self.current_linear, self.current_linear_accel = self._smooth_velocity(
            current=self.current_linear,
            target=self.target_linear,
            current_accel=self.current_linear_accel,
            max_accel=self.max_linear_accel,
            max_decel=self.max_linear_decel,
            max_jerk=self.max_linear_jerk if self.use_jerk_limit else None,
            dt=dt
        )
        
        # Smooth angular velocity
        self.current_angular, self.current_angular_accel = self._smooth_velocity(
            current=self.current_angular,
            target=self.target_angular,
            current_accel=self.current_angular_accel,
            max_accel=self.max_angular_accel,
            max_decel=self.max_angular_decel,
            max_jerk=self.max_angular_jerk if self.use_jerk_limit else None,
            dt=dt
        )
        
        # Publish smoothed command
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.pub.publish(msg)

    def _smooth_velocity(self, current, target, current_accel, 
                         max_accel, max_decel, max_jerk, dt):
        """
        Smooth velocity transition with acceleration and optional jerk limits.
        
        Returns: (new_velocity, new_acceleration)
        """
        error = target - current
        
        if abs(error) < 1e-6:
            # Already at target
            return target, 0.0
        
        # Determine if accelerating or decelerating
        # Decelerating = moving toward zero OR overshooting target
        if abs(target) < abs(current) or (error * current < 0):
            accel_limit = max_decel
        else:
            accel_limit = max_accel
        
        # Desired acceleration to reach target
        desired_accel = error / dt  # Would reach target in one step
        
        # Clamp acceleration
        desired_accel = np.clip(desired_accel, -accel_limit, accel_limit)
        
        # Apply jerk limit (limit how fast acceleration can change)
        if max_jerk is not None:
            accel_change = desired_accel - current_accel
            max_accel_change = max_jerk * dt
            accel_change = np.clip(accel_change, -max_accel_change, max_accel_change)
            new_accel = current_accel + accel_change
        else:
            new_accel = desired_accel
        
        # Update velocity
        new_velocity = current + new_accel * dt
        
        # Don't overshoot target
        if (error > 0 and new_velocity > target) or (error < 0 and new_velocity < target):
            new_velocity = target
            new_accel = 0.0
        
        return new_velocity, new_accel


def main():
    rclpy.init()
    node = VelocitySmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()