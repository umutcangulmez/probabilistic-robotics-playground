#!/usr/bin/env python3
"""
ES-EKF Experiment Executor
A working ROS2 node that runs experiments and collects metrics.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped

import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from typing import List, Optional, Dict
import time
import csv
from datetime import datetime


# ============================================================
# Data Classes
# ============================================================

@dataclass
class EKFConfig:
    """Configuration for EKF"""
    sigma_accel: float = 0.05
    sigma_gyro: float = 0.005
    sigma_range: float = 0.1
    sigma_bearing: float = 0.05
    landmarks: Dict = None

    def __post_init__(self):
        if self.landmarks is None:
            self.landmarks = {
                1: np.array([6.0, 6.0, 0.5]),
                2: np.array([6.0, -6.0, 0.5]),
                3: np.array([10.0, 0.0, 0.5]),
                4: np.array([0.0, 8.0, 0.5])
            }


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


# Predefined scenarios
SCENARIOS = {
    "baseline": Scenario(
        name="baseline",
        duration=60.0,
        linear_speed=0.5
    ),
    "high_imu_noise": Scenario(
        name="high_imu_noise",
        duration=60.0,
        imu_accel_noise=0.2,
        imu_gyro_noise=0.02
    ),
    "high_visual_noise": Scenario(
        name="high_visual_noise",
        duration=60.0,
        visual_range_noise=0.4,
        visual_bearing_noise=0.2
    ),
    "fast_motion": Scenario(
        name="fast_motion",
        duration=60.0,
        linear_speed=1.0,
        angular_speed=0.6
    ),
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
        """Compute RMSE, NEES, and drift"""
        if len(self.gt_positions) < 10 or len(self.est_positions) < 10:
            self.get_logger().info(f"1111111111111")
            return None

        # Align by nearest timestamp
        gt_times = np.array(self.gt_times)
        aligned_gt_pos = []
        aligned_est_pos = []
        aligned_gt_quat = []
        aligned_est_quat = []
        # Debug: print time ranges
        print(f"GT times: min={min(self.gt_times):.1f}, max={max(self.gt_times):.1f}")
        print(f"EST times: min={min(self.est_times):.1f}, max={max(self.est_times):.1f}")

        for i, t in enumerate(self.est_times):
            idx = np.argmin(np.abs(gt_times - t))
            if abs(gt_times[idx] - t) < 0.1:
                aligned_gt_pos.append(self.gt_positions[idx])
                aligned_est_pos.append(self.est_positions[i])
                aligned_gt_quat.append(self.gt_orientations[idx])
                aligned_est_quat.append(self.est_orientations[i])

        if len(aligned_gt_pos) < 10:
            self.get_logger().info(f"222222222222222")
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

        # Drift (error per meter traveled)
        diffs = np.diff(gt_pos, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        total_distance = np.sum(distances)
        final_error = np.linalg.norm(pos_err[-1])
        drift_rate = final_error / total_distance if total_distance > 0 else 0

        # NEES (simplified - position only)
        nees_values = []
        if len(self.est_covariances) > 0:
            for i, (gt_p, est_p) in enumerate(zip(aligned_gt_pos, aligned_est_pos)):
                if i < len(self.est_covariances):
                    err = est_p - gt_p
                    P_pos = self.est_covariances[i][:3, :3]
                    try:
                        nees = err @ np.linalg.inv(P_pos) @ err
                        nees_values.append(nees)
                    except:
                        pass

        nees_mean = np.mean(nees_values) if nees_values else 0

        return {
            'position_rmse': position_rmse,
            'yaw_rmse': np.degrees(yaw_rmse),
            'drift_rate': drift_rate,
            'final_error': final_error,
            'total_distance': total_distance,
            'nees_mean': nees_mean,
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
        """Get velocity command at time t"""
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
    """
    ROS2 Node that runs ES-EKF experiments.

    Usage:
        ros2 run your_package experiment_executor
    """

    def __init__(self):
        super().__init__('experiment_executor')
        self.ekf_msg = None

        # Parameters
        self.declare_parameter('scenarios', ['baseline'])
        self.declare_parameter('num_runs', 1)
        self.declare_parameter('results_file', 'ekf_results.csv')

        self.scenario_names = self.get_parameter('scenarios').value
        self.num_runs = self.get_parameter('num_runs').value
        self.results_file = self.get_parameter('results_file').value

        # TF for ground truth
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/vehicle_blue/cmd_vel', 10)

        # Subscribers
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_callback, 50
        )

        # State
        self.metrics = MetricsCollector()
        self.trajectory = None
        self.is_running = False
        self.start_time = None
        self.current_scenario = None
        self.all_results = []

        # Covariance storage (will be set by EKF node if available)
        self.latest_covariance = None

        self.get_logger().info("Experiment Executor initialized")
        self.get_logger().info(f"Scenarios to run: {self.scenario_names}")

        # Start experiments after a delay
        self.create_timer(2.0, self.start_experiments, callback_group=None)
        self._experiments_started = False

    def ekf_callback(self, msg: Odometry):
        """Collect EKF estimates"""
        self.ekf_msg = msg
        if not self.is_running:
            self.get_logger().info('EKF callback triggered but not running', throttle_duration_sec=1.0)
            return
        self.get_logger().info('EKF callback triggered and is running', throttle_duration_sec=1.0)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.metrics.add_estimate(t, pos, quat, self.latest_covariance)

        # Also collect ground truth
        self.collect_ground_truth()
    def collect_estimate(self):
        """Get latest EKF estimate"""
        if self.ekf_msg is not None:
            msg = self.ekf_msg
            t = time.time()  # Use wall clock time to match GT
            pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            quat = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            self.metrics.add_estimate(t, pos, quat, None)

    def collect_ground_truth(self):
        """Get ground truth from TF"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'vehicle_blue/chassis', Time(), timeout=Duration(seconds=0.1)
            )
            t = time.time()  # Use wall clock time

            pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            quat = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])

            self.metrics.add_ground_truth(t, pos, quat)

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)

    def start_experiments(self):
        """Start running experiments"""
        if self._experiments_started:
            return
        self._experiments_started = True

        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting experiments...")
        self.get_logger().info("=" * 60)

        for scenario_name in self.scenario_names:
            if scenario_name not in SCENARIOS:
                self.get_logger().warn(f"Unknown scenario: {scenario_name}")
                continue

            scenario = SCENARIOS[scenario_name]

            for run_id in range(self.num_runs):
                self.run_single_experiment(scenario, run_id)

        self.save_results()
        self.print_summary()

        self.get_logger().info("All experiments completed!")

    def run_single_experiment(self, scenario: Scenario, run_id: int):
        """Run a single experiment"""
        self.get_logger().info(f"\n--- Running: {scenario.name} (run {run_id + 1}/{self.num_runs}) ---")

        # Reset
        self.metrics.reset()
        self.current_scenario = scenario
        self.trajectory = TrajectoryGenerator(
            linear_speed=scenario.linear_speed,
            angular_speed=scenario.angular_speed
        )

        # Wait for systems to stabilize
        time.sleep(1.0)

        # Run experiment
        self.is_running = True
        start_time = time.time()  # Use wall clock time


        while rclpy.ok():
            elapsed = time.time() - start_time

            if elapsed >= scenario.duration:
                break

            # Send velocity command
            twist = self.trajectory.get_command(elapsed)
            self.cmd_vel_pub.publish(twist)
            # Collect ground truth
            self.collect_ground_truth()
            self.collect_estimate()

            # Process callbacks
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.001)

        # Stop vehicle
        self.is_running = False
        self.cmd_vel_pub.publish(Twist())

        self.get_logger().info(f"  Experiment finished after {elapsed:.1f}s")
        self.get_logger().info(f"  GT samples: {len(self.metrics.gt_positions)}, EST samples: {len(self.metrics.est_positions)}")

    # Compute metrics
        results = self.metrics.compute_metrics()

        if results:
            results['scenario'] = scenario.name
            results['run'] = run_id
            self.all_results.append(results)

            self.get_logger().info(
                f"  RMSE: {results['position_rmse']:.3f}m | "
                f"Yaw: {results['yaw_rmse']:.2f}° | "
                f"Drift: {results['drift_rate']:.4f} m/m"
            )
        else:
            self.get_logger().warn("  Failed to compute metrics (insufficient data)")

    def save_results(self):
        """Save results to CSV"""
        if not self.all_results:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{self.results_file}"

        fieldnames = ['scenario', 'run', 'position_rmse', 'yaw_rmse',
                      'drift_rate', 'final_error', 'total_distance',
                      'nees_mean', 'num_samples']

        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.all_results)

        self.get_logger().info(f"Results saved to: {filename}")

    def print_summary(self):
        """Print summary of all results"""
        if not self.all_results:
            return

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("EXPERIMENT SUMMARY")
        self.get_logger().info("=" * 60)

        # Group by scenario
        scenarios = {}
        for r in self.all_results:
            name = r['scenario']
            if name not in scenarios:
                scenarios[name] = []
            scenarios[name].append(r)

        for name, runs in scenarios.items():
            rmse_vals = [r['position_rmse'] for r in runs]
            yaw_vals = [r['yaw_rmse'] for r in runs]
            drift_vals = [r['drift_rate'] for r in runs]

            self.get_logger().info(f"\n{name}:")
            self.get_logger().info(f"  Position RMSE: {np.mean(rmse_vals):.3f} ± {np.std(rmse_vals):.3f} m")
            self.get_logger().info(f"  Yaw RMSE:      {np.mean(yaw_vals):.2f} ± {np.std(yaw_vals):.2f} °")
            self.get_logger().info(f"  Drift rate:    {np.mean(drift_vals):.4f} ± {np.std(drift_vals):.4f} m/m")


def main():
    rclpy.init()

    executor = ExperimentExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.cmd_vel_pub.publish(Twist())  # Stop vehicle
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()