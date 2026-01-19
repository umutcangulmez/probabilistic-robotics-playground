#!/usr/bin/env python3
"""
ES-EKF Experiment Executor
Corrected for Sim Time Synchronization
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.clock import ClockType

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from typing import List, Optional, Dict
import csv
from datetime import datetime

# ============================================================
# Data Classes
# ============================================================
# ... (Keep EKFConfig and Scenario classes exactly as they were) ...
@dataclass
class Scenario:
    """Experiment scenario definition"""
    name: str
    duration: float = 10.0
    linear_speed: float = 0.5
    angular_speed: float = 0.3
    imu_accel_noise: float = 0.05
    imu_gyro_noise: float = 0.005
    visual_range_noise: float = 0.1
    visual_bearing_noise: float = 0.05

SCENARIOS = {
    "baseline": Scenario(name="baseline", duration=60.0, linear_speed=0.5),
    "high_imu_noise": Scenario(name="high_imu_noise", duration=60.0, imu_accel_noise=0.2, imu_gyro_noise=0.02),
    "high_visual_noise": Scenario(name="high_visual_noise", duration=60.0, visual_range_noise=0.4, visual_bearing_noise=0.2),
    "fast_motion": Scenario(name="fast_motion", duration=60.0, linear_speed=1.0, angular_speed=0.6),
}

# ============================================================
# Metrics Collector
# ============================================================
class MetricsCollector:
    """Collects and computes experiment metrics"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.gt_positions = []
        self.gt_orientations = []
        self.gt_times = []
        self.est_positions = []
        self.est_orientations = []
        self.est_times = []
        self.est_covariances = []

    def add_ground_truth(self, t: float, pos: np.ndarray, quat: np.ndarray):
        self.gt_times.append(t)
        self.gt_positions.append(pos.copy())
        self.gt_orientations.append(quat.copy())

    def add_estimate(self, t: float, pos: np.ndarray, quat: np.ndarray, cov: np.ndarray = None):
        self.est_times.append(t)
        self.est_positions.append(pos.copy())
        self.est_orientations.append(quat.copy())
        if cov is not None:
            self.est_covariances.append(cov.copy())

    def compute_metrics(self) -> Dict:
        if len(self.gt_positions) < 10 or len(self.est_positions) < 10:
            print("Not enough data points")
            return None

        # Align by nearest timestamp
        gt_times = np.array(self.gt_times)
        aligned_gt_pos = []
        aligned_est_pos = []
        aligned_gt_quat = []
        aligned_est_quat = []

        # Debug: print time ranges
        print(f"GT times: min={min(self.gt_times):.2f}, max={max(self.gt_times):.2f}")
        print(f"EST times: min={min(self.est_times):.2f}, max={max(self.est_times):.2f}")

        for i, t in enumerate(self.est_times):
            # Find closest GT timestamp
            idx = np.argmin(np.abs(gt_times - t))
            time_diff = abs(gt_times[idx] - t)

            # Allow small tolerance (0.1s) for sync
            if time_diff < 0.1:
                aligned_gt_pos.append(self.gt_positions[idx])
                aligned_est_pos.append(self.est_positions[i])
                aligned_gt_quat.append(self.gt_orientations[idx])
                aligned_est_quat.append(self.est_orientations[i])

        if len(aligned_gt_pos) < 10:
            print("Failed to align data (timestamps don't overlap)")
            return None

        gt_pos = np.array(aligned_gt_pos)
        est_pos = np.array(aligned_est_pos)

        # Position RMSE
        pos_err = est_pos - gt_pos
        position_rmse = np.sqrt(np.mean(np.sum(pos_err**2, axis=1)))

        # Yaw RMSE
        yaw_errors = []
        for q_gt, q_est in zip(aligned_gt_quat, aligned_est_quat):
            r_gt = R.from_quat(q_gt)
            r_est = R.from_quat(q_est)
            r_err = r_gt.inv() * r_est
            yaw_err = r_err.as_euler('xyz')[2]
            yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
            yaw_errors.append(yaw_err)
        yaw_rmse = np.sqrt(np.mean(np.array(yaw_errors)**2))

        # Drift calculation
        diffs = np.diff(gt_pos, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        total_distance = np.sum(distances)
        final_error = np.linalg.norm(pos_err[-1])
        drift_rate = final_error / total_distance if total_distance > 0 else 0

        # NEES
        nees_values = []
        if len(self.est_covariances) > 0:
            # Note: aligning covariances requires index tracking, simplified here
            pass

        return {
            'position_rmse': position_rmse,
            'yaw_rmse': np.degrees(yaw_rmse),
            'drift_rate': drift_rate,
            'final_error': final_error,
            'total_distance': total_distance,
            'nees_mean': 0.0, # Placeholder
            'num_samples': len(aligned_gt_pos)
        }

# ============================================================
# Trajectory Generator
# ============================================================
class TrajectoryGenerator:
    """Generates square trajectory velocity commands"""
    def __init__(self, linear_speed: float = 0.5, angular_speed: float = 0.3):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.side_length = 4.0
        self.side_time = self.side_length / linear_speed
        self.turn_time = (np.pi / 2) / angular_speed

    def get_command(self, t: float) -> Twist:
        cycle_time = 4 * (self.side_time + self.turn_time)
        t_in_cycle = t % cycle_time
        phase_time = self.side_time + self.turn_time
        t_in_phase = t_in_cycle % phase_time

        twist = Twist()
        if t_in_phase < self.side_time:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        return twist

# ============================================================
# Experiment Executor Node
# ============================================================
class ExperimentExecutor(Node):
    def __init__(self):
        super().__init__('experiment_executor')
        self.ekf_msg = None
        self.gt_msg = None

        # Subscribing to the remapped topic from Launch file
        self.gt_sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self.gt_callback, 50
        )

        self.declare_parameter('scenarios', ['baseline'])
        self.declare_parameter('num_runs', 1)
        self.declare_parameter('results_file', 'ekf_results.csv')
        self.declare_parameter('use_sim_time', True) # Default to true for experiments

        self.scenario_names = self.get_parameter('scenarios').value
        self.num_runs = self.get_parameter('num_runs').value
        self.results_file = self.get_parameter('results_file').value

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/vehicle_blue/cmd_vel', 10)
        self.ekf_sub = self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 50)

        self.metrics = MetricsCollector()
        self.trajectory = None
        self.is_running = False
        self.current_scenario = None
        self.all_results = []
        self.latest_covariance = None

        self.get_logger().info("Experiment Executor initialized")
        self.create_timer(2.0, self.start_experiments, callback_group=None)
        self._experiments_started = False

    def gt_callback(self, msg: Odometry):
        # Normalize quaternion
        q = msg.pose.pose.orientation
        if q.w < 0:
            msg.pose.pose.orientation.x = -q.x
            msg.pose.pose.orientation.y = -q.y
            msg.pose.pose.orientation.z = -q.z
            msg.pose.pose.orientation.w = -q.w
        self.gt_msg = msg

    def ekf_callback(self, msg: Odometry):
        self.ekf_msg = msg
        # We rely on the control loop to sample data to avoid duplicates

    def collect_estimate(self):
        """Get latest EKF estimate using SIM TIME"""
        if self.ekf_msg is not None:
            msg = self.ekf_