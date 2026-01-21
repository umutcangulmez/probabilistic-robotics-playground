#!/usr/bin/env python3
"""
data_logger.py

Simple data logger node that records EKF and ground truth data to CSV.
Run this alongside your simulation, drive the robot, then stop.

Usage:
    ros2 run visual_inertial_nav_es_ekf data_logger --ros-args -p output_dir:=/path/to/output -p experiment_name:=imu_vision_10lm
"""

import os
import csv
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/ros2_ws/experiment_data'))
        self.declare_parameter('experiment_name', 'experiment')
        self.declare_parameter('gt_frame', 'turtlebot3_waffle/base_footprint')
        self.declare_parameter('target_frame', 'odom')

        self.output_dir = self.get_parameter('output_dir').value
        self.experiment_name = self.get_parameter('experiment_name').value
        self.gt_frame = self.get_parameter('gt_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        # Create output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.output_dir, f"{self.experiment_name}_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)

        # TF listener for ground truth
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data storage
        self.ekf_data = []
        self.gt_data = []
        self.metrics_data = []
        self.visual_data = []

        # Subscribers
        self.sub_ekf = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 50)
        self.sub_rmse = self.create_subscription(
            Float64, '/ekf/rmse', self.rmse_callback, 50)
        self.sub_nees = self.create_subscription(
            Float64, '/ekf/nees', self.nees_callback, 50)
        self.sub_error = self.create_subscription(
            Float64, '/ekf/pose_error', self.error_callback, 50)
        self.sub_visual = self.create_subscription(
            PointStamped, '/visual_measurement', self.visual_callback, 50)

        # Periodic ground truth lookup
        self.create_timer(0.05, self.gt_timer_callback)  # 20 Hz

        # Periodic status
        self.create_timer(5.0, self.status_callback)

        # Start time
        self.start_time = None
        self.last_rmse = 0.0
        self.last_nees = 0.0
        self.last_error = 0.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("DATA LOGGER STARTED")
        self.get_logger().info(f"Output: {self.session_dir}")
        self.get_logger().info(f"Experiment: {self.experiment_name}")
        self.get_logger().info("Drive the robot, then Ctrl+C to save data")
        self.get_logger().info("=" * 60)

    def get_timestamp(self, msg_stamp) -> float:
        """Get relative timestamp in seconds."""
        t = msg_stamp.sec + msg_stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def ekf_callback(self, msg: Odometry):
        t = self.get_timestamp(msg.header.stamp)

        # Extract quaternion to yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2 * np.arctan2(qz, qw)

        # Extract covariance
        cov = np.array(msg.pose.covariance).reshape(6, 6)

        self.ekf_data.append({
            'time': t,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'cov_xx': cov[0, 0],
            'cov_yy': cov[1, 1],
            'cov_xy': cov[0, 1],
            'cov_yaw': cov[5, 5],
        })

    def gt_timer_callback(self):
        """Lookup ground truth from TF."""
        if self.start_time is None:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.gt_frame,
                Time(),
                timeout=Duration(seconds=0.01)
            )

            now = self.get_clock().now()
            rel_time = (now.nanoseconds * 1e-9) - self.start_time

            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2 * np.arctan2(qz, qw)

            self.gt_data.append({
                'time': rel_time,
                'x': t.transform.translation.x,
                'y': t.transform.translation.y,
                'z': t.transform.translation.z,
                'yaw': yaw,
            })

        except tf2_ros.TransformException:
            pass

    def rmse_callback(self, msg: Float64):
        self.last_rmse = msg.data

    def nees_callback(self, msg: Float64):
        self.last_nees = msg.data

    def error_callback(self, msg: Float64):
        self.last_error = msg.data
        if self.start_time is not None:
            now = self.get_clock().now()
            rel_time = (now.nanoseconds * 1e-9) - self.start_time
            self.metrics_data.append({
                'time': rel_time,
                'rmse': self.last_rmse,
                'nees': self.last_nees,
                'error': self.last_error,
            })

    def visual_callback(self, msg: PointStamped):
        if self.start_time is None:
            return
        t = self.get_timestamp(msg.header.stamp)
        self.visual_data.append({
            'time': t,
            'bearing': msg.point.x,
            'range': msg.point.y,
            'landmark_id': int(msg.point.z),
        })

    def status_callback(self):
        self.get_logger().info(
            f"Recording... EKF: {len(self.ekf_data)}, GT: {len(self.gt_data)}, "
            f"Visual: {len(self.visual_data)}, RMSE: {self.last_rmse:.3f}m"
        )

    def save_data(self):
        """Save all data to CSV files."""
        self.get_logger().info("Saving data...")

        # Save EKF data
        if self.ekf_data:
            path = os.path.join(self.session_dir, 'ekf_trajectory.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.ekf_data[0].keys())
                writer.writeheader()
                writer.writerows(self.ekf_data)
            self.get_logger().info(f"Saved {len(self.ekf_data)} EKF samples to {path}")

        # Save GT data
        if self.gt_data:
            path = os.path.join(self.session_dir, 'ground_truth.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.gt_data[0].keys())
                writer.writeheader()
                writer.writerows(self.gt_data)
            self.get_logger().info(f"Saved {len(self.gt_data)} GT samples to {path}")

        # Save metrics
        if self.metrics_data:
            path = os.path.join(self.session_dir, 'metrics.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.metrics_data[0].keys())
                writer.writeheader()
                writer.writerows(self.metrics_data)
            self.get_logger().info(f"Saved {len(self.metrics_data)} metric samples to {path}")

        # Save visual measurements
        if self.visual_data:
            path = os.path.join(self.session_dir, 'visual_measurements.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.visual_data[0].keys())
                writer.writeheader()
                writer.writerows(self.visual_data)
            self.get_logger().info(f"Saved {len(self.visual_data)} visual measurements to {path}")

        # Save summary
        summary = {
            'experiment_name': self.experiment_name,
            'duration_sec': self.ekf_data[-1]['time'] if self.ekf_data else 0,
            'ekf_samples': len(self.ekf_data),
            'gt_samples': len(self.gt_data),
            'visual_updates': len(self.visual_data),
            'final_rmse': self.last_rmse,
            'final_nees': self.last_nees,
        }

        import json
        path = os.path.join(self.session_dir, 'summary.json')
        with open(path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info("=" * 60)
        self.get_logger().info("DATA SAVED SUCCESSFULLY")
        self.get_logger().info(f"Directory: {self.session_dir}")
        self.get_logger().info(f"Duration: {summary['duration_sec']:.1f}s")
        self.get_logger().info(f"Final RMSE: {self.last_rmse:.4f}m")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()