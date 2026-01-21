#!/usr/bin/env python3
"""
Trajectory Publisher - Predefined trajectories for ES-EKF experiments

Publishes velocity commands to follow predefined paths:
  - square: Square pattern
  - circle: Circular pattern
  - figure8: Figure-8 pattern
  - lawnmower: Back-and-forth coverage pattern

Usage:
    ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args -p trajectory:=square
    ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args -p trajectory:=circle -p radius:=3.0
    ros2 run visual_inertial_nav_es_ekf trajectory_publisher --ros-args -p trajectory:=figure8 -p scale:=4.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import List
import time


class TrajectorySegment:
    """A segment of trajectory: either move straight or rotate in place."""

    def __init__(self, linear: float = 0.0, angular: float = 0.0,
                 duration: float = 0.0, description: str = ""):
        self.linear = linear      # m/s
        self.angular = angular    # rad/s
        self.duration = duration  # seconds
        self.description = description

    def __repr__(self):
        return f"Segment({self.description}: v={self.linear:.2f}, ω={self.angular:.2f}, t={self.duration:.1f}s)"


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Parameters
        self.declare_parameter('trajectory', 'square')
        self.declare_parameter('linear_speed', 0.3)      # m/s
        self.declare_parameter('angular_speed', 0.3)     # rad/s
        self.declare_parameter('side_length', 6.0)       # For square
        self.declare_parameter('radius', 3.0)            # For circle
        self.declare_parameter('scale', 4.0)             # For figure8
        self.declare_parameter('loops', 1)               # Number of times to repeat
        self.declare_parameter('output_topic', '/cmd_vel_raw')
        self.declare_parameter('wait_before_start', 3.0) # Seconds to wait before starting

        # Load parameters
        self.trajectory_name = self.get_parameter('trajectory').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.side_length = self.get_parameter('side_length').value
        self.radius = self.get_parameter('radius').value
        self.scale = self.get_parameter('scale').value
        self.loops = self.get_parameter('loops').value
        self.wait_before_start = self.get_parameter('wait_before_start').value
        output_topic = self.get_parameter('output_topic').value

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)

        # State
        self.segments: List[TrajectorySegment] = []
        self.current_segment_idx = 0
        self.segment_start_time = None
        self.is_running = False
        self.is_finished = False
        self.current_loop = 0

        # Generate trajectory
        self.generate_trajectory()

        # Timer for publishing commands (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Start time tracking
        self.node_start_time = time.time()

        self.print_info()

    def print_info(self):
        """Print trajectory information."""
        total_duration = sum(s.duration for s in self.segments) * self.loops

        self.get_logger().info("=" * 60)
        self.get_logger().info("TRAJECTORY PUBLISHER")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Trajectory: {self.trajectory_name}")
        self.get_logger().info(f"Linear speed: {self.linear_speed} m/s")
        self.get_logger().info(f"Angular speed: {self.angular_speed} rad/s")
        self.get_logger().info(f"Loops: {self.loops}")
        self.get_logger().info(f"Total segments: {len(self.segments)}")
        self.get_logger().info(f"Estimated duration: {total_duration:.1f}s")
        self.get_logger().info(f"Starting in {self.wait_before_start:.1f} seconds...")
        self.get_logger().info("=" * 60)

        # Print segments
        for i, seg in enumerate(self.segments):
            self.get_logger().info(f"  [{i+1}] {seg}")

    def generate_trajectory(self):
        """Generate trajectory segments based on type."""
        traj_type = self.trajectory_name.lower()

        if traj_type == "square":
            self.segments = self.generate_square()
        elif traj_type == "circle":
            self.segments = self.generate_circle()
        elif traj_type == "figure8":
            self.segments = self.generate_figure8()
        elif traj_type == "lawnmower":
            self.segments = self.generate_lawnmower()
        else:
            self.get_logger().error(f"Unknown trajectory: {self.trajectory_name}")
            self.get_logger().info("Available: square, circle, figure8, lawnmower")
            self.segments = []

    def generate_square(self) -> List[TrajectorySegment]:
        """
        Generate square trajectory.

        Pattern: Move forward, turn 90°, repeat 4 times.
        """
        segments = []

        # Time to travel one side
        side_time = self.side_length / self.linear_speed

        # Time to turn 90 degrees
        turn_time = (math.pi / 2) / self.angular_speed

        for i in range(4):
            # Move forward
            segments.append(TrajectorySegment(
                linear=self.linear_speed,
                angular=0.0,
                duration=side_time,
                description=f"Side {i+1}: Forward {self.side_length:.1f}m"
            ))

            # Turn left 90 degrees
            segments.append(TrajectorySegment(
                linear=0.0,
                angular=self.angular_speed,
                duration=turn_time,
                description=f"Corner {i+1}: Turn 90°"
            ))

        return segments

    def generate_circle(self) -> List[TrajectorySegment]:
        """
        Generate circular trajectory.

        Uses constant linear and angular velocity to trace a circle.
        For a circle: v = ω * r, so ω = v / r
        """
        segments = []

        # Calculate angular velocity for desired radius
        # v = ω * r => ω = v / r
        angular_vel = self.linear_speed / self.radius

        # Time for full circle: circumference / speed = 2πr / v
        circle_time = (2 * math.pi * self.radius) / self.linear_speed

        segments.append(TrajectorySegment(
            linear=self.linear_speed,
            angular=angular_vel,
            duration=circle_time,
            description=f"Circle: r={self.radius:.1f}m, circumference={2*math.pi*self.radius:.1f}m"
        ))

        return segments

    def generate_figure8(self) -> List[TrajectorySegment]:
        """
        Generate figure-8 trajectory.

        Two circles in opposite directions, connected.
        """
        segments = []

        # Use half the scale as radius for each loop
        radius = self.scale / 2.0

        # Angular velocity for the curves
        angular_vel = self.linear_speed / radius

        # Time for full circle
        circle_time = (2 * math.pi * radius) / self.linear_speed

        # First loop (turn left/counterclockwise)
        segments.append(TrajectorySegment(
            linear=self.linear_speed,
            angular=angular_vel,
            duration=circle_time,
            description=f"Loop 1 (CCW): r={radius:.1f}m"
        ))

        # Second loop (turn right/clockwise) - opposite direction
        segments.append(TrajectorySegment(
            linear=self.linear_speed,
            angular=-angular_vel,
            duration=circle_time,
            description=f"Loop 2 (CW): r={radius:.1f}m"
        ))

        return segments

    def generate_lawnmower(self) -> List[TrajectorySegment]:
        """
        Generate lawnmower/boustrophedon pattern.

        Good for covering an area systematically.
        """
        segments = []

        num_rows = 4
        row_length = self.side_length
        row_spacing = 1.5  # meters between rows

        row_time = row_length / self.linear_speed
        turn_time = (math.pi / 2) / self.angular_speed
        spacing_time = row_spacing / self.linear_speed

        for i in range(num_rows):
            # Move along row
            segments.append(TrajectorySegment(
                linear=self.linear_speed,
                angular=0.0,
                duration=row_time,
                description=f"Row {i+1}: Forward {row_length:.1f}m"
            ))

            if i < num_rows - 1:  # Not last row
                # Turn direction alternates
                turn_dir = 1 if i % 2 == 0 else -1

                # Turn 90°
                segments.append(TrajectorySegment(
                    linear=0.0,
                    angular=turn_dir * self.angular_speed,
                    duration=turn_time,
                    description=f"Turn {'left' if turn_dir > 0 else 'right'} 90°"
                ))

                # Move to next row
                segments.append(TrajectorySegment(
                    linear=self.linear_speed,
                    angular=0.0,
                    duration=spacing_time,
                    description=f"Spacing: {row_spacing:.1f}m"
                ))

                # Turn 90° again (same direction)
                segments.append(TrajectorySegment(
                    linear=0.0,
                    angular=turn_dir * self.angular_speed,
                    duration=turn_time,
                    description=f"Turn {'left' if turn_dir > 0 else 'right'} 90°"
                ))

        return segments

    def timer_callback(self):
        """Main control loop."""
        current_time = time.time()

        # Wait before starting
        if current_time - self.node_start_time < self.wait_before_start:
            # Publish zero velocity while waiting
            self.publish_cmd(0.0, 0.0)
            remaining = self.wait_before_start - (current_time - self.node_start_time)
            if int(remaining) != int(remaining + 0.02):  # Log once per second
                self.get_logger().info(f"Starting in {int(remaining)+1} seconds...")
            return

        # Start trajectory
        if not self.is_running and not self.is_finished:
            self.is_running = True
            self.segment_start_time = current_time
            self.get_logger().info(">>> TRAJECTORY STARTED <<<")

        # Check if finished
        if self.is_finished:
            self.publish_cmd(0.0, 0.0)
            return

        # No segments
        if not self.segments:
            self.is_finished = True
            return

        # Get current segment
        segment = self.segments[self.current_segment_idx]
        elapsed = current_time - self.segment_start_time

        # Check if segment is complete
        if elapsed >= segment.duration:
            # Move to next segment
            self.current_segment_idx += 1

            # Check if loop complete
            if self.current_segment_idx >= len(self.segments):
                self.current_loop += 1

                if self.current_loop >= self.loops:
                    # All loops complete
                    self.is_finished = True
                    self.is_running = False
                    self.publish_cmd(0.0, 0.0)
                    self.get_logger().info("=" * 60)
                    self.get_logger().info(">>> TRAJECTORY COMPLETE <<<")
                    self.get_logger().info("=" * 60)
                    return
                else:
                    # Start next loop
                    self.current_segment_idx = 0
                    self.get_logger().info(f"--- Starting loop {self.current_loop + 1}/{self.loops} ---")

            # Start new segment
            self.segment_start_time = current_time
            segment = self.segments[self.current_segment_idx]
            self.get_logger().info(
                f"[{self.current_segment_idx + 1}/{len(self.segments)}] {segment.description}"
            )

        # Publish command for current segment
        self.publish_cmd(segment.linear, segment.angular)

    def publish_cmd(self, linear: float, angular: float):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        # Send stop command
        msg = Twist()
        node.cmd_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()