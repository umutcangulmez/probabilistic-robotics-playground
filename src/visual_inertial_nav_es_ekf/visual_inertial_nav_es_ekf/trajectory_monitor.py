"""
Trajectory Monitor with Improved Time Alignment and NEES

Priority 2 Fixes:
1. Tries TF at odom_msg.header.stamp first (proper time alignment)
2. Falls back to latest if time-sync fails, but records fallback count
3. Adds NEES (Normalized Estimation Error Squared) computation
4. Reports ATE RMSE over time-aligned trajectories

NEES Computation:
  NEES = (x_est - x_gt)^T @ P^(-1) @ (x_est - x_gt)
  For a consistent filter, NEES should follow chi-squared distribution.
  For 2D position, E[NEES] ≈ 2, and 95% bounds are [0.05, 5.99]
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from collections import deque


class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')

        # --- Parameters ---
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('gt_frame', 'base_footprint')
        self.declare_parameter('ekf_topic', '/ekf_odom')
        self.declare_parameter('path_max_len', 5000)
        self.declare_parameter('nees_window', 100)  # Window for running NEES average
        self.ekf_buffer = deque(maxlen=100)  # Store recent EKF poses

        self.target_frame = self.get_parameter('target_frame').value
        self.gt_frame = self.get_parameter('gt_frame').value
        self.path_max_len = int(self.get_parameter('path_max_len').value)
        self.nees_window = int(self.get_parameter('nees_window').value)

        # --- TF Listener ---
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.sub_ekf = self.create_subscription(
            Odometry, 
            self.get_parameter('ekf_topic').value, 
            self.ekf_callback, 
            10
        )

        # --- Publishers ---
        self.pub_gt_path = self.create_publisher(Path, '/ground_truth/path', 10)
        self.pub_error = self.create_publisher(Float64, '/ekf/pose_error', 10)
        self.pub_rmse = self.create_publisher(Float64, '/ekf/rmse', 10)
        self.pub_nees = self.create_publisher(Float64, '/ekf/nees', 10)
        self.pub_nees_avg = self.create_publisher(Float64, '/ekf/nees_avg', 10)

        # --- State ---
        self.gt_path_msg = Path()
        self.gt_path_msg.header.frame_id = self.target_frame
        
        # ATE RMSE tracking
        self.sq_error_sum = 0.0
        self.sample_count = 0
        self.aligned_sample_count = 0  # Properly time-aligned samples
        self.fallback_count = 0         # Samples using "latest" fallback
        
        # NEES tracking
        self.nees_history = []
        
        # Last EKF covariance (for NEES)
        self.last_P_pos = None
        self.create_timer(0.1, self.process_matches)

        self.create_timer(5.0, self.report_stats)
        self.get_logger().info("Trajectory Monitor initialized (with NEES). Waiting for EKF data...")


    def ekf_callback(self, odom_msg: Odometry):
        """Just buffer the EKF message"""
        self.ekf_buffer.append(odom_msg)
    def process_matches(self):
        """Match buffered EKF poses with available TF"""
        while self.ekf_buffer:
            odom_msg = self.ekf_buffer[0]
            ekf_time = Time.from_msg(odom_msg.header.stamp)

            # Check if TF is available for this timestamp
            try:
                if self.tf_buffer.can_transform(
                        self.target_frame,
                        self.gt_frame,
                        ekf_time,
                        timeout=Duration(seconds=0.0)
                ):
                    # TF available - process this message
                    self.ekf_buffer.popleft()
                    self._process_matched_pose(odom_msg)
                else:
                    # TF not yet available - check if it's too old
                    now = self.get_clock().now()
                    age = (now - ekf_time).nanoseconds / 1e9

                    if age > 1.0:  # Older than 1 second, skip it
                        self.ekf_buffer.popleft()
                        self.fallback_count += 1
                    else:
                        # Wait for TF to arrive
                        break
            except tf2_ros.TransformException:
                self.ekf_buffer.popleft()
                break
    def _process_matched_pose(self, odom_msg: Odometry):
        # 1. Get EKF Position
        pos_est = odom_msg.pose.pose.position
        
        # 2. Extract covariance for NEES (position only: indices 0,1 for x,y)
        # Odometry covariance is 6x6 (x,y,z,roll,pitch,yaw)
        cov = np.array(odom_msg.pose.covariance).reshape(6, 6)
        P_pos_xy = cov[0:2, 0:2]  # 2x2 position covariance
        
        # 3. Get Ground Truth Position via TF
        # Priority 2 Fix: Try time-aligned first, then fallback to latest
        gt_x, gt_y, gt_z, gt_rot, time_aligned = self._get_gt_transform(odom_msg)
        
        if gt_x is None:
            return  # TF not available at all
        
        # Track alignment statistics
        if time_aligned:
            self.aligned_sample_count += 1
        else:
            self.fallback_count += 1

        # 4. Calculate Error (Euclidean Distance) - 2D for planar robot
        dx = pos_est.x - gt_x
        dy = pos_est.y - gt_y
        
        error_sq = dx*dx + dy*dy
        error_dist = math.sqrt(error_sq)

        # 5. Update ATE RMSE
        self.sq_error_sum += error_sq
        self.sample_count += 1
        rmse = math.sqrt(self.sq_error_sum / self.sample_count)

        # 6. Compute NEES (2D position)
        nees = self._compute_nees(np.array([dx, dy]), P_pos_xy)

        # 7. Publish Metrics
        err_msg = Float64()
        err_msg.data = error_dist
        self.pub_error.publish(err_msg)

        rmse_msg = Float64()
        rmse_msg.data = rmse
        self.pub_rmse.publish(rmse_msg)
        
        if nees is not None:
            nees_msg = Float64()
            nees_msg.data = nees
            self.pub_nees.publish(nees_msg)
            
            # Running average NEES
            self.nees_history.append(nees)
            if len(self.nees_history) > self.nees_window:
                self.nees_history.pop(0)
            
            nees_avg = np.mean(self.nees_history)
            nees_avg_msg = Float64()
            nees_avg_msg.data = nees_avg
            self.pub_nees_avg.publish(nees_avg_msg)

        # 8. Build and Publish GT Path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_stamped.header.frame_id = self.target_frame
        pose_stamped.pose.position.x = gt_x
        pose_stamped.pose.position.y = gt_y
        pose_stamped.pose.position.z = gt_z
        pose_stamped.pose.orientation = gt_rot

        self.gt_path_msg.poses.append(pose_stamped)
        
        if len(self.gt_path_msg.poses) > self.path_max_len:
            self.gt_path_msg.poses.pop(0)
            
        self.gt_path_msg.header.stamp = odom_msg.header.stamp
        self.pub_gt_path.publish(self.gt_path_msg)

    def _get_gt_transform(self, odom_msg):
        """
        Get ground truth transform with proper time alignment.
        
        Returns: (x, y, z, rotation, time_aligned)
        """
        # First, try to get transform at the exact EKF message time
        ekf_time = Time.from_msg(odom_msg.header.stamp)
        
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.gt_frame, 
                ekf_time,
                timeout=Duration(seconds=0.05)  # Short timeout for exact time
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation,
                True  # Time aligned
            )
        except tf2_ros.TransformException:
            pass  # Fall through to latest
        
        # Fallback: Get latest available transform
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.gt_frame, 
                Time(),  # Latest
                timeout=Duration(seconds=0.1)
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation,
                False  # Not time aligned (fallback)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF unavailable: {ex}', throttle_duration_sec=1.0)
            return (None, None, None, None, False)

    def _compute_nees(self, error: np.ndarray, P: np.ndarray) -> float:
        """
        Compute Normalized Estimation Error Squared (NEES).
        
        NEES = error^T @ P^(-1) @ error
        
        For a consistent estimator with dim=2:
          E[NEES] = 2
          95% bounds: [0.05, 5.99] (chi-squared)
        
        Args:
            error: 2D position error [dx, dy]
            P: 2x2 position covariance
        
        Returns:
            NEES value, or None if covariance is singular
        """
        # Check if covariance is valid
        if P is None or np.any(np.isnan(P)) or np.any(np.isinf(P)):
            return None
        
        # Add small regularization for numerical stability
        P_reg = P + np.eye(2) * 1e-10
        
        try:
            P_inv = np.linalg.inv(P_reg)
            nees = error.T @ P_inv @ error
            return float(nees)
        except np.linalg.LinAlgError:
            return None

    def report_stats(self):
        """Periodic statistics report"""
        if self.sample_count == 0:
            return
            
        rmse = math.sqrt(self.sq_error_sum / self.sample_count)
        align_pct = 100.0 * self.aligned_sample_count / self.sample_count if self.sample_count > 0 else 0
        nees_avg = np.mean(self.nees_history) if self.nees_history else 0
        
        self.get_logger().info(
            f"Stats: RMSE={rmse:.4f}m, "
            f"NEES_avg={nees_avg:.2f} (expect~2), "
            f"Aligned={align_pct:.1f}%, "
            f"Samples={self.sample_count}"
        )
        
        # NEES consistency check
        if len(self.nees_history) >= 20:
            if nees_avg < 0.05:
                self.get_logger().warn("NEES too low (<0.05): Filter may be overconfident!")
            elif nees_avg > 5.99:
                self.get_logger().warn("NEES too high (>5.99): Filter may be inconsistent!")


def main():
    rclpy.init()
    node = TrajectoryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()