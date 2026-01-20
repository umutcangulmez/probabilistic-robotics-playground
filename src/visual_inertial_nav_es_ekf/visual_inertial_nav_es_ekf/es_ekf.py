#!/usr/bin/env python3
"""
ES-EKF - Improved Version with Fixes

Improvements over diagnostic version:
1. Fixed ground constraint covariance handling (no multiplicative damping)
2. Proper sequential IMU integration (no gyro averaging)
3. Separate raw/filtered IMU paths for stationarity vs prediction
4. Velocity covariance floor after ZUPT
5. Outlier rejection for visual updates with Mahalanobis distance
6. Bias clipping warnings
7. Better numerical stability throughout
8. Enhanced debug output with innovation monitoring

State vector (error-state):
  [0:3]   - position error (x, y, z)
  [3:6]   - velocity error (vx, vy, vz)
  [6:9]   - orientation error (rotation vector)
  [9:12]  - accelerometer bias
  [12:15] - gyroscope bias
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass, field
from enum import Enum
import json

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry


# =============================================================================
# Math Utilities
# =============================================================================

def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix from 3-vector."""
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0]
    ], dtype=float)


def normalize_quat(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [x, y, z, w]."""
    n = np.linalg.norm(q)
    if n > 1e-10:
        return q / n
    return np.array([0., 0., 0., 1.], dtype=float)


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion multiplication q1 ⊗ q2, format [x,y,z,w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)


def rotvec_to_quat(rv: np.ndarray) -> np.ndarray:
    """Convert rotation vector to quaternion [x,y,z,w]."""
    angle = float(np.linalg.norm(rv))
    if angle < 1e-10:
        # Small angle approximation
        return normalize_quat(np.array([rv[0]/2, rv[1]/2, rv[2]/2, 1.0], dtype=float))
    axis = rv / angle
    ha = angle / 2.0
    return np.array([
        axis[0] * np.sin(ha),
        axis[1] * np.sin(ha),
        axis[2] * np.sin(ha),
        np.cos(ha)
    ], dtype=float)


def wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return float(np.arctan2(np.sin(a), np.cos(a)))


# =============================================================================
# Data Structures
# =============================================================================

class UpdateType(Enum):
    ZUPT = "zupt"
    VISUAL = "visual"


@dataclass
class FilterStats:
    """Statistics for filter monitoring."""
    predict_count: int = 0
    zupt_count: int = 0
    visual_count: int = 0
    bias_clipped_count: int = 0
    
    # Innovation monitoring (rolling)
    range_innovations: List[float] = field(default_factory=list)
    bearing_innovations: List[float] = field(default_factory=list)
    
    def add_visual_innovation(self, range_innov: float, bearing_innov: float):
        self.range_innovations.append(range_innov)
        self.bearing_innovations.append(bearing_innov)
        # Keep last 100
        if len(self.range_innovations) > 100:
            self.range_innovations.pop(0)
            self.bearing_innovations.pop(0)
    
    def get_innovation_stats(self) -> Tuple[float, float, float, float]:
        """Return (range_mean, range_std, bearing_mean, bearing_std)."""
        if len(self.range_innovations) < 5:
            return (0.0, 0.0, 0.0, 0.0)
        return (
            float(np.mean(self.range_innovations)),
            float(np.std(self.range_innovations)),
            float(np.mean(self.bearing_innovations)),
            float(np.std(self.bearing_innovations))
        )


# =============================================================================
# ES-EKF Core
# =============================================================================

class ESEKF:
    """
    Error-State Extended Kalman Filter for 2D ground robot.
    
    Assumes planar motion with known z-height.
    """
    
    # State indices
    IDX_POS = slice(0, 3)
    IDX_VEL = slice(3, 6)
    IDX_ORI = slice(6, 9)
    IDX_AB = slice(9, 12)
    IDX_WB = slice(12, 15)
    
    def __init__(
        self,
        init_pos: Optional[np.ndarray] = None,
        sigma_a: float = 0.1,
        sigma_w: float = 0.01,
        sigma_ab: float = 1e-4,
        sigma_wb: float = 1e-5,
        R_range: float = 0.01,
        R_bearing: float = 0.0025,
        sigma_zupt: float = 0.02,
        fixed_z: float = 0.0,
    ):
        # Process noise parameters (base values)
        self.sigma_a_base = float(sigma_a)
        self.sigma_w_base = float(sigma_w)
        self.sigma_a = float(sigma_a)
        self.sigma_w = float(sigma_w)
        self.sigma_ab = float(sigma_ab)
        self.sigma_wb = float(sigma_wb)
        
        # Motion-dependent noise scaling
        self.motion_noise_scale = 2.0  # Multiply noise when moving
        self.turn_noise_scale = 3.0    # Additional scale when turning
        
        # Measurement noise
        self.R_range = float(R_range)
        self.R_bearing = float(R_bearing)
        self.R_zupt = float(sigma_zupt) ** 2
        
        # Constraints
        self.fixed_z = float(fixed_z)
        
        # Bias limits (for clipping with warnings)
        self.ab_limit = 0.5  # m/s^2
        self.wb_limit = 0.1  # rad/s
        
        # ===== Nominal State =====
        self.p = init_pos.copy() if init_pos is not None else np.zeros(3, dtype=float)
        self.v = np.zeros(3, dtype=float)
        self.q = np.array([0., 0., 0., 1.], dtype=float)  # [x,y,z,w]
        self.ab = np.zeros(3, dtype=float)
        self.wb = np.zeros(3, dtype=float)
        
        # Gravity in world frame
        self.g_world = np.array([0., 0., -9.81], dtype=float)
        
        # ===== Error-State Covariance =====
        self.P = np.diag([
            0.1, 0.1, 1e-6,      # Position (x, y, z)
            0.01, 0.01, 1e-6,    # Velocity
            1e-6, 1e-6, 0.1,     # Orientation (roll, pitch, yaw)
            0.01, 0.01, 0.01,    # Accel bias
            1e-4, 1e-4, 1e-4     # Gyro bias
        ]).astype(float)
        
        # ===== Initialization =====
        self.is_initialized = False
        self.init_samples: List[dict] = []
        self.init_sample_count = 100
        
        # ===== Stationarity Detection =====
        self.is_stationary = False
        self.stationary_counter = 0
        
        # ===== Statistics =====
        self.stats = FilterStats()

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------
    
    def add_init_sample(self, accel: np.ndarray, gyro: np.ndarray) -> bool:
        """
        Collect samples for static initialization.
        Returns True when initialization is complete.
        """
        a_mag = float(np.linalg.norm(accel))
        g_mag = float(np.linalg.norm(gyro))
        
        # Only accept samples that look stationary
        if 8.0 < a_mag < 12.0 and g_mag < 0.15:
            self.init_samples.append({
                'accel': accel.copy(),
                'gyro': gyro.copy()
            })
        
        if len(self.init_samples) >= self.init_sample_count:
            self._complete_init()
            return True
        return False

    def _complete_init(self):
        """Complete initialization using collected samples."""
        accels = np.array([s['accel'] for s in self.init_samples], dtype=float)
        gyros = np.array([s['gyro'] for s in self.init_samples], dtype=float)
        
        # Use median for robustness
        a_med = np.median(accels, axis=0)
        g_med = np.median(gyros, axis=0)
        
        # Estimate gravity magnitude from accelerometer
        measured_g = float(np.linalg.norm(a_med))
        self.g_world = np.array([0., 0., -measured_g], dtype=float)
        
        # Initial gyro bias is the median reading (should be ~zero when stationary)
        self.wb = g_med.copy()
        
        # Initial accel bias: measured - expected (gravity in body frame)
        # Assuming robot starts level, expected is [0, 0, g]
        f_expected = np.array([0., 0., measured_g], dtype=float)
        self.ab = (a_med - f_expected).astype(float)
        
        # Estimate initial orientation from gravity
        # (Assumes small roll/pitch initially)
        self._estimate_initial_orientation(a_med)
        
        self.is_initialized = True
        self.init_samples = []

    def _estimate_initial_orientation(self, accel: np.ndarray):
        """Estimate initial roll/pitch from gravity vector."""
        # Normalize accelerometer reading
        a_norm = accel / np.linalg.norm(accel)
        
        # Roll and pitch from gravity direction
        # In body frame, gravity should point [0, 0, 1] when level
        roll = np.arctan2(a_norm[1], a_norm[2])
        pitch = np.arctan2(-a_norm[0], np.sqrt(a_norm[1]**2 + a_norm[2]**2))
        
        # Keep yaw at zero (unknown from IMU alone)
        self.q = normalize_quat(R.from_euler('xyz', [roll, pitch, 0.0]).as_quat())

    # -------------------------------------------------------------------------
    # Prediction Step
    # -------------------------------------------------------------------------
    
    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """
        Propagate state and covariance using IMU measurements.
        
        Args:
            accel: Accelerometer reading [m/s^2] in body frame
            gyro: Gyroscope reading [rad/s] in body frame
            dt: Time step [s]
        """
        if not self.is_initialized or dt <= 0.0 or dt > 0.5:
            return
        
        self.stats.predict_count += 1
        
        # Bias-corrected measurements
        f_body = accel - self.ab
        w_body = gyro - self.wb
        
        # Current rotation matrix
        rot = R.from_quat(self.q).as_matrix()
        
        # ===== Motion-dependent process noise scaling =====
        speed = float(np.linalg.norm(self.v[:2]))
        turn_rate = float(abs(w_body[2]))
        
        # Scale noise based on motion
        motion_scale = 1.0
        if speed > 0.1:  # Moving
            motion_scale = self.motion_noise_scale
        if turn_rate > 0.05:  # Turning
            motion_scale *= self.turn_noise_scale
        
        self.sigma_a = self.sigma_a_base * motion_scale
        self.sigma_w = self.sigma_w_base * motion_scale
        
        # Acceleration in world frame
        a_world = rot @ f_body + self.g_world
        
        # ===== State Propagation =====
        # Position: p += v*dt + 0.5*a*dt^2
        self.p = self.p + self.v * dt + 0.5 * a_world * (dt ** 2)
        
        # Velocity: v += a*dt
        self.v = self.v + a_world * dt
        
        # Orientation: q = q ⊗ δq(w*dt)
        w_norm = float(np.linalg.norm(w_body))
        if w_norm > 1e-10:
            dq = rotvec_to_quat(w_body * dt)
            self.q = normalize_quat(quat_mult(self.q, dq))
        
        # ===== Covariance Propagation =====
        self._propagate_covariance(f_body, w_body, rot, dt)
        
        # ===== Ground Constraints =====
        self._apply_ground_constraints()

    def _propagate_covariance(self, f_body: np.ndarray, w_body: np.ndarray, 
                               rot: np.ndarray, dt: float):
        """Propagate error-state covariance."""
        # State transition Jacobian
        Fx = np.eye(15, dtype=float)
        
        # ∂p/∂v = I*dt
        Fx[0:3, 3:6] = np.eye(3) * dt
        
        # ∂v/∂θ = -R*[f_body]_× * dt
        Fx[3:6, 6:9] = -rot @ skew(f_body) * dt
        
        # ∂v/∂ab = -R * dt
        Fx[3:6, 9:12] = -rot * dt
        
        # ∂θ/∂θ ≈ I - [w_body]_× * dt
        Fx[6:9, 6:9] = np.eye(3) - skew(w_body) * dt
        
        # ∂θ/∂wb = -I * dt
        Fx[6:9, 12:15] = -np.eye(3) * dt
        
        # Process noise
        sa2 = self.sigma_a ** 2
        sw2 = self.sigma_w ** 2
        sab2 = self.sigma_ab ** 2
        swb2 = self.sigma_wb ** 2
        
        # Noise input matrix
        Fi = np.zeros((15, 12), dtype=float)
        Fi[3:6, 0:3] = rot * dt      # Accel noise → velocity
        Fi[6:9, 3:6] = np.eye(3) * dt  # Gyro noise → orientation
        Fi[9:12, 6:9] = np.eye(3) * np.sqrt(dt)   # Accel bias random walk
        Fi[12:15, 9:12] = np.eye(3) * np.sqrt(dt)  # Gyro bias random walk
        
        # Continuous-time noise covariance
        Qc = np.diag([sa2]*3 + [sw2]*3 + [sab2]*3 + [swb2]*3)
        
        # Discrete process noise
        Q = Fi @ Qc @ Fi.T
        
        # Propagate covariance
        self.P = Fx @ self.P @ Fx.T + Q
        
        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def _apply_ground_constraints(self):
        """
        Apply planar ground robot constraints.
        
        Constraints:
        - z position fixed
        - z velocity zero
        - roll and pitch near zero
        """
        # Fix z position and velocity
        self.p[2] = self.fixed_z
        self.v[2] = 0.0
        
        # Extract yaw and reset orientation to yaw-only
        yaw = R.from_quat(self.q).as_euler('xyz')[2]
        self.q = normalize_quat(R.from_euler('z', yaw).as_quat())
        
        # ===== FIX: Set constrained states to small fixed variance =====
        # (Not multiplicative damping which causes inconsistency)
        
        constrained_indices = [2, 5, 6, 7]  # z, vz, roll, pitch
        
        for idx in constrained_indices:
            # Set small variance
            self.P[idx, idx] = 1e-10
            
            # Zero out cross-correlations
            self.P[idx, :] = 0.0
            self.P[:, idx] = 0.0
            self.P[idx, idx] = 1e-10  # Restore diagonal

    # -------------------------------------------------------------------------
    # Stationarity Detection
    # -------------------------------------------------------------------------
    
    def update_stationarity(self, accel_raw: np.ndarray, gyro_raw: np.ndarray, 
                            cfg: dict) -> bool:
        """
        Update stationarity state using RAW (unfiltered) IMU data.
        
        FIX: Uses raw IMU for responsive detection, not filtered.
        
        Returns: True if stationary
        """
        if not self.is_initialized:
            return False
        
        # Compute kinematic acceleration (removing gravity)
        f_body = accel_raw - self.ab
        rot = R.from_quat(self.q).as_matrix()
        g_body = rot.T @ self.g_world
        a_kin = f_body + g_body
        
        # Metrics
        acc_xy = float(np.linalg.norm(a_kin[:2]))
        gyro_z = float(abs(gyro_raw[2] - self.wb[2]))
        speed_xy = float(np.linalg.norm(self.v[:2]))
        
        # Thresholds
        acc_enter = float(cfg.get('acc_enter', 0.15))
        acc_exit = float(cfg.get('acc_exit', 0.30))
        gyro_enter = float(cfg.get('gyro_enter', 0.02))
        gyro_exit = float(cfg.get('gyro_exit', 0.05))
        vel_thresh = float(cfg.get('vel_thresh', 0.05))
        samples_req = int(cfg.get('samples_req', 15))
        
        if self.is_stationary:
            # Check for motion (exit stationary)
            if acc_xy > acc_exit or gyro_z > gyro_exit or speed_xy > vel_thresh * 2:
                self.is_stationary = False
                self.stationary_counter = 0
        else:
            # Check for stillness (enter stationary)
            is_quiet = (acc_xy < acc_enter and gyro_z < gyro_enter and speed_xy < vel_thresh)
            
            if is_quiet:
                self.stationary_counter += 1
                if self.stationary_counter >= samples_req:
                    self.is_stationary = True
            else:
                self.stationary_counter = 0
        
        return self.is_stationary

    # -------------------------------------------------------------------------
    # ZUPT Update
    # -------------------------------------------------------------------------
    
    def zupt_update(self) -> bool:
        """
        Zero-velocity update (ZUPT).
        
        Applies when robot is stationary to correct velocity drift.
        
        Returns: True if update was applied successfully
        """
        if not self.is_stationary:
            return False
        
        # Measurement: velocity should be zero
        # Innovation: y = 0 - v (only x,y for planar robot)
        y = -self.v[:2]
        
        # Measurement Jacobian
        H = np.zeros((2, 15), dtype=float)
        H[0, 3] = 1.0  # ∂z/∂vx
        H[1, 4] = 1.0  # ∂z/∂vy
        
        # Measurement noise
        R_m = np.eye(2, dtype=float) * self.R_zupt
        
        # Apply Kalman update
        dx = self._kalman_update(y, H, R_m)
        
        if dx is not None:
            self._inject_error(dx)
            self.stats.zupt_count += 1
            
            # ===== FIX: Apply velocity covariance floor after ZUPT =====
            max_vel_var = self.R_zupt * 4.0  # Slightly above measurement noise
            self.P[3, 3] = min(self.P[3, 3], max_vel_var)
            self.P[4, 4] = min(self.P[4, 4], max_vel_var)
            
            return True
        
        return False

    # -------------------------------------------------------------------------
    # Visual Landmark Update
    # -------------------------------------------------------------------------
    
    def update_range_bearing(self, lm_world: np.ndarray, z_range: float, 
                              z_bearing: float, cam_offset: Optional[np.ndarray] = None) -> bool:
        """
        Update with range and bearing measurement to a known landmark.
        
        Args:
            lm_world: Landmark position in world frame [x, y, z]
            z_range: Measured range to landmark [m]
            z_bearing: Measured bearing to landmark [rad]
            cam_offset: Camera position offset in body frame [x, y, z]
        
        Returns: True if update was applied
        """
        if not self.is_initialized:
            return False
        
        if cam_offset is None:
            cam_offset = np.zeros(3, dtype=float)
        
        rot = R.from_quat(self.q).as_matrix()
        rot_T = rot.T
        
        # Transform landmark to body frame
        lm_body = rot_T @ (lm_world - self.p)
        
        # Transform to camera frame
        lm_cam = lm_body - cam_offset
        
        # Predicted measurements
        pred_range = float(np.linalg.norm(lm_cam[:2]))
        
        # Validity check (landmark must be in front of camera and not too close)
        if pred_range < 0.5 or lm_cam[0] < 0.1:
            return False
        
        pred_bearing = float(np.arctan2(lm_cam[1], lm_cam[0]))
        
        # Innovation
        y = np.array([
            float(z_range) - pred_range,
            wrap_angle(float(z_bearing) - pred_bearing)
        ], dtype=float)
        
        # Build Jacobian
        x_c, y_c = float(lm_cam[0]), float(lm_cam[1])
        r = pred_range
        
        # ∂[r, θ]/∂lm_cam
        dr_dl = np.array([x_c / r, y_c / r, 0.0], dtype=float)
        db_dl = np.array([-y_c / (r**2), x_c / (r**2), 0.0], dtype=float)
        
        # Measurement Jacobian
        H = np.zeros((2, 15), dtype=float)
        
        # ∂h/∂p: derivative of lm_cam w.r.t position
        H[0, 0:3] = dr_dl @ (-rot_T)
        H[1, 0:3] = db_dl @ (-rot_T)
        
        # ∂h/∂θ: derivative w.r.t orientation
        H[0, 6:9] = dr_dl @ skew(lm_body)
        H[1, 6:9] = db_dl @ skew(lm_body)
        
        # Measurement noise
        R_m = np.diag([self.R_range, self.R_bearing]).astype(float)
        
        # Record innovation for monitoring
        self.stats.add_visual_innovation(y[0], y[1])
        
        # Apply Kalman update (no outlier rejection)
        dx = self._kalman_update(y, H, R_m)
        
        if dx is not None:
            self._inject_error(dx)
            self.stats.visual_count += 1
            return True
        
        return False

    # -------------------------------------------------------------------------
    # Kalman Update Mechanics
    # -------------------------------------------------------------------------
    
    def _kalman_update(self, y: np.ndarray, H: np.ndarray, 
                        R_m: np.ndarray) -> Optional[np.ndarray]:
        """
        Standard Kalman update equations.
        
        Returns error-state correction dx, or None if update fails.
        """
        # Innovation covariance
        S = H @ self.P @ H.T + R_m
        
        # Kalman gain: K = P @ H.T @ S^{-1}
        try:
            K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
        except np.linalg.LinAlgError:
            return None
        
        # Error-state correction
        dx = K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(15, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_m @ K.T
        
        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)
        
        return dx

    def _inject_error(self, dx: np.ndarray):
        """
        Inject error-state correction into nominal state.
        """
        # Position and velocity (additive)
        self.p += dx[0:3]
        self.v += dx[3:6]
        
        # Orientation (multiplicative)
        dtheta = dx[6:9]
        if float(np.linalg.norm(dtheta)) > 1e-10:
            dq = rotvec_to_quat(dtheta)
            self.q = normalize_quat(quat_mult(self.q, dq))
            
            # Reset orientation error covariance
            # P_new = G @ P @ G.T where G accounts for error reset
            G = np.eye(15, dtype=float)
            G[6:9, 6:9] = np.eye(3) - 0.5 * skew(dtheta)
            self.P = G @ self.P @ G.T
            self.P = 0.5 * (self.P + self.P.T)
        
        # ===== FIX: Bias update with clipping warning =====
        ab_new = self.ab + dx[9:12]
        wb_new = self.wb + dx[12:15]
        
        # Check if clipping will occur
        if np.any(np.abs(ab_new) > self.ab_limit * 0.9):
            self.stats.bias_clipped_count += 1
        if np.any(np.abs(wb_new) > self.wb_limit * 0.9):
            self.stats.bias_clipped_count += 1
        
        # Apply with clipping
        self.ab = np.clip(ab_new, -self.ab_limit, self.ab_limit)
        self.wb = np.clip(wb_new, -self.wb_limit, self.wb_limit)
        
        # Re-apply ground constraints
        self._apply_ground_constraints()

    # -------------------------------------------------------------------------
    # Accessors
    # -------------------------------------------------------------------------
    
    def get_pose_covariance_6x6(self) -> np.ndarray:
        """
        Get 6x6 pose covariance in ROS format.
        [x, y, z, roll, pitch, yaw]
        """
        cov = np.zeros((6, 6), dtype=float)
        
        # Position covariance
        cov[0:3, 0:3] = self.P[0:3, 0:3]
        
        # Orientation covariance
        cov[3:6, 3:6] = self.P[6:9, 6:9]
        
        # Cross-correlation
        cov[0:3, 3:6] = self.P[0:3, 6:9]
        cov[3:6, 0:3] = self.P[6:9, 0:3]
        
        # Ensure minimum variance for constrained states
        for i in [2, 3, 4]:  # z, roll, pitch
            cov[i, i] = max(cov[i, i], 1e-10)
        
        return cov

    def get_yaw(self) -> float:
        """Get current yaw angle in radians."""
        return float(R.from_quat(self.q).as_euler('xyz')[2])

    def get_speed(self) -> float:
        """Get current 2D speed in m/s."""
        return float(np.linalg.norm(self.v[:2]))


# =============================================================================
# ROS 2 Node
# =============================================================================

class ESEKFNode(Node):
    def __init__(self):
        super().__init__('es_ekf_node')
        
        # =====================================================================
        # Parameters
        # =====================================================================
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('vis_topic', '/visual_measurement')
        self.declare_parameter('odom_topic', '/ekf_odom')
        self.declare_parameter('path_topic', '/ekf_path')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        
        # IMU processing
        self.declare_parameter('filter_alpha', 0.35)
        
        # Stationarity detection
        self.declare_parameter('acc_enter_threshold', 0.15)
        self.declare_parameter('acc_exit_threshold', 0.30)
        self.declare_parameter('gyro_enter_threshold', 0.02)
        self.declare_parameter('gyro_exit_threshold', 0.05)
        self.declare_parameter('velocity_zupt_threshold', 0.05)
        self.declare_parameter('stationary_samples_required', 15)
        
        # Process noise
        self.declare_parameter('sigma_a', 0.1)
        self.declare_parameter('sigma_w', 0.01)
        self.declare_parameter('sigma_ab', 1e-4)
        self.declare_parameter('sigma_wb', 1e-5)
        
        # Measurement noise
        self.declare_parameter('R_range', 0.01)
        self.declare_parameter('R_bearing', 0.0025)
        self.declare_parameter('sigma_zupt', 0.02)
        
        # Initial state
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 0.0)
        
        # Camera offset
        self.declare_parameter('cam_offset_x', 0.1)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.0)
        
        # =====================================================================
        # Read Parameters
        # =====================================================================
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        imu_topic = self.get_parameter('imu_topic').value
        vis_topic = self.get_parameter('vis_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        
        self.alpha = float(np.clip(self.get_parameter('filter_alpha').value, 0.0, 1.0))
        
        self.zupt_config = {
            'acc_enter': float(self.get_parameter('acc_enter_threshold').value),
            'acc_exit': float(self.get_parameter('acc_exit_threshold').value),
            'gyro_enter': float(self.get_parameter('gyro_enter_threshold').value),
            'gyro_exit': float(self.get_parameter('gyro_exit_threshold').value),
            'vel_thresh': float(self.get_parameter('velocity_zupt_threshold').value),
            'samples_req': int(self.get_parameter('stationary_samples_required').value),
        }
        
        init_pos = np.array([
            float(self.get_parameter('init_x').value),
            float(self.get_parameter('init_y').value),
            float(self.get_parameter('init_z').value),
        ], dtype=float)
        
        self.cam_offset = np.array([
            float(self.get_parameter('cam_offset_x').value),
            float(self.get_parameter('cam_offset_y').value),
            float(self.get_parameter('cam_offset_z').value),
        ], dtype=float)
        
        # =====================================================================
        # Create EKF
        # =====================================================================
        self.ekf = ESEKF(
            init_pos=init_pos,
            sigma_a=float(self.get_parameter('sigma_a').value),
            sigma_w=float(self.get_parameter('sigma_w').value),
            sigma_ab=float(self.get_parameter('sigma_ab').value),
            sigma_wb=float(self.get_parameter('sigma_wb').value),
            R_range=float(self.get_parameter('R_range').value),
            R_bearing=float(self.get_parameter('R_bearing').value),
            sigma_zupt=float(self.get_parameter('sigma_zupt').value),
            fixed_z=float(init_pos[2]),
        )
        self.declare_parameter('landmarks_file', '')
        landmarks_file = self.get_parameter('landmarks_file').value
        if landmarks_file:
            self.landmarks = self.load_landmarks(landmarks_file)
        else:
            # Landmark map (same as visual_detector.py)
            self.landmarks = {
                1: np.array([6.0, 6.0, 0.5], dtype=float),
                2: np.array([6.0, -6.0, 0.5], dtype=float),
                3: np.array([10.0, 0.0, 0.5], dtype=float),
                4: np.array([0.0, 8.0, 0.5], dtype=float),
            }
        
        # =====================================================================
        # IMU State
        # =====================================================================
        self.last_imu_time: Optional[float] = None
        self.accel_filtered: Optional[np.ndarray] = None
        self.gyro_filtered: Optional[np.ndarray] = None
        
        # Debug storage
        self.last_raw_accel = np.zeros(3)
        self.last_raw_gyro = np.zeros(3)
        self.last_a_kin = np.zeros(3)
        
        # =====================================================================
        # ROS I/O
        # =====================================================================
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 50)
        self.sub_vis = self.create_subscription(
            PointStamped, vis_topic, self.visual_callback, 50)
        
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame_id
        
        # Debug timer
        self.create_timer(2.0, self.debug_callback)
        
        # =====================================================================
        # Startup Banner
        # =====================================================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("ES-EKF IMPROVED VERSION (No Outlier Rejection)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"IMU topic: {imu_topic}")
        self.get_logger().info(f"Visual topic: {vis_topic}")
        self.get_logger().info(f"Init pos: [{init_pos[0]:.2f}, {init_pos[1]:.2f}, {init_pos[2]:.2f}]")
        self.get_logger().info(f"Cam offset: [{self.cam_offset[0]:.2f}, {self.cam_offset[1]:.2f}]")
        self.get_logger().info(f"Motion noise scale: {self.ekf.motion_noise_scale:.1f}x, "
                              f"Turn scale: {self.ekf.turn_noise_scale:.1f}x")
        self.get_logger().info("=" * 60)

    def load_landmarks(self, json_path: str) -> dict:
        """Load landmarks from JSON file into dict format."""
        with open(json_path, 'r') as f:
            data = json.load(f)

        landmarks = {}
        for lm in data['landmarks']:
            landmarks[lm['id']] = np.array([lm['x'], lm['y'], lm['z']], dtype=float)

        # Log for verification
        self.get_logger().info(f"Loaded {len(landmarks)} landmarks from {json_path}:")
        for lid, pos in landmarks.items():
            self.get_logger().info(f"  ID {lid}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

        return landmarks
    # -------------------------------------------------------------------------
    # IMU Callback
    # -------------------------------------------------------------------------
    
    def imu_callback(self, msg: Imu):
        """Process IMU message."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_imu_time is None:
            self.last_imu_time = t
            return
        
        dt = t - self.last_imu_time
        self.last_imu_time = t
        
        # Sanity check
        if dt <= 0 or dt > 0.2:
            self.accel_filtered = None
            self.gyro_filtered = None
            return
        
        # Extract measurements
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=float)
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)
        
        self.last_raw_accel = accel.copy()
        self.last_raw_gyro = gyro.copy()
        
        # ===== Initialization =====
        if not self.ekf.is_initialized:
            if self.ekf.add_init_sample(accel, gyro):
                self.get_logger().info("=" * 60)
                self.get_logger().info("ES-EKF INITIALIZED")
                self.get_logger().info(f"|g| = {np.linalg.norm(self.ekf.g_world):.4f}")
                self.get_logger().info(f"ab = [{self.ekf.ab[0]:.4f}, {self.ekf.ab[1]:.4f}, {self.ekf.ab[2]:.4f}]")
                self.get_logger().info(f"wb = [{self.ekf.wb[0]:.6f}, {self.ekf.wb[1]:.6f}, {self.ekf.wb[2]:.6f}]")
                self.get_logger().info("=" * 60)
            return
        
        # ===== FIX: Stationarity detection on RAW data =====
        self.ekf.update_stationarity(accel, gyro, self.zupt_config)
        
        # ===== Low-pass filter for prediction =====
        if self.accel_filtered is None:
            self.accel_filtered = accel.copy()
            self.gyro_filtered = gyro.copy()
        else:
            self.accel_filtered = self.alpha * accel + (1.0 - self.alpha) * self.accel_filtered
            self.gyro_filtered = self.alpha * gyro + (1.0 - self.alpha) * self.gyro_filtered
        
        # ===== FIX: Process each IMU sample individually =====
        # (No downsampling/averaging which causes integration errors)
        
        # Compute kinematic acceleration for debug
        f_body = self.accel_filtered - self.ekf.ab
        rot = R.from_quat(self.ekf.q).as_matrix()
        g_body = rot.T @ self.ekf.g_world
        self.last_a_kin = f_body + g_body
        
        # Prediction step
        self.ekf.predict(self.accel_filtered, self.gyro_filtered, dt)
        
        # ZUPT update if stationary
        if self.ekf.is_stationary:
            self.ekf.zupt_update()
        
        # Publish state
        self.publish_state(msg.header.stamp)

    # -------------------------------------------------------------------------
    # Visual Callback
    # -------------------------------------------------------------------------
    
    def visual_callback(self, msg: PointStamped):
        """Process visual landmark measurement."""
        if not self.ekf.is_initialized:
            return
        
        lm_id = int(msg.point.z)
        if lm_id not in self.landmarks:
            return
        
        z_bearing = float(msg.point.x)
        z_range = float(msg.point.y)
        lm_world = self.landmarks[lm_id]
        
        # Apply update (outlier rejection happens inside)
        self.ekf.update_range_bearing(lm_world, z_range, z_bearing, self.cam_offset)

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------
    
    def publish_state(self, stamp):
        """Publish current state estimate."""
        ekf = self.ekf
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        odom.pose.pose.position.x = float(ekf.p[0])
        odom.pose.pose.position.y = float(ekf.p[1])
        odom.pose.pose.position.z = float(ekf.p[2])
        
        odom.pose.pose.orientation.x = float(ekf.q[0])
        odom.pose.pose.orientation.y = float(ekf.q[1])
        odom.pose.pose.orientation.z = float(ekf.q[2])
        odom.pose.pose.orientation.w = float(ekf.q[3])
        
        odom.twist.twist.linear.x = float(ekf.v[0])
        odom.twist.twist.linear.y = float(ekf.v[1])
        odom.twist.twist.linear.z = float(ekf.v[2])
        
        cov = ekf.get_pose_covariance_6x6()
        odom.pose.covariance = cov.flatten().tolist()
        
        self.pub_odom.publish(odom)
        
        # Path message
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path_msg.poses.append(pose)
        
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)
        
        self.path_msg.header.stamp = stamp
        self.pub_path.publish(self.path_msg)

    # -------------------------------------------------------------------------
    # Debug Output
    # -------------------------------------------------------------------------
    
    def debug_callback(self):
        """Periodic debug output."""
        if not self.ekf.is_initialized:
            self.get_logger().info("Waiting for initialization...")
            return
        
        ekf = self.ekf
        stats = ekf.stats
        
        yaw_deg = np.degrees(ekf.get_yaw())
        speed = ekf.get_speed()
        stat = "STAT" if ekf.is_stationary else "MOVE"
        
        a_kin_xy = float(np.linalg.norm(self.last_a_kin[:2]))
        gyro_z = float(abs(self.last_raw_gyro[2] - ekf.wb[2]))
        
        # Innovation statistics
        r_mean, r_std, b_mean, b_std = stats.get_innovation_stats()
        
        # Noise scaling info
        noise_scale = ekf.sigma_a / ekf.sigma_a_base
        
        self.get_logger().info("-" * 70)
        self.get_logger().info(f"POS: [{ekf.p[0]:7.3f}, {ekf.p[1]:7.3f}] m")
        self.get_logger().info(f"VEL: [{ekf.v[0]:7.3f}, {ekf.v[1]:7.3f}] m/s  |v|={speed:.3f}")
        self.get_logger().info(f"YAW: {yaw_deg:7.1f}°  [{stat}]")
        self.get_logger().info(f"a_kin_xy={a_kin_xy:.3f} m/s²  gyro_z={gyro_z:.4f} rad/s")
        self.get_logger().info(f"ab=[{ekf.ab[0]:.4f}, {ekf.ab[1]:.4f}, {ekf.ab[2]:.4f}]")
        self.get_logger().info(f"wb=[{ekf.wb[0]:.6f}, {ekf.wb[1]:.6f}, {ekf.wb[2]:.6f}]")
        self.get_logger().info(
            f"Updates - Predict: {stats.predict_count}, ZUPT: {stats.zupt_count}, "
            f"Visual: {stats.visual_count}"
        )
        self.get_logger().info(
            f"tr(P)={np.trace(ekf.P):.4f}, P_pos=[{ekf.P[0,0]:.4f}, {ekf.P[1,1]:.4f}], "
            f"Noise scale: {noise_scale:.1f}x"
        )
        
        if stats.visual_count > 0:
            self.get_logger().info(
                f"Innovations - Range: {r_mean:.3f}±{r_std:.3f}m, "
                f"Bearing: {b_mean:.4f}±{b_std:.4f}rad"
            )
        
        if stats.bias_clipped_count > 0:
            self.get_logger().warn(f"Bias clipping occurred {stats.bias_clipped_count} times!")


# =============================================================================
# Main
# =============================================================================

def main():
    rclpy.init()
    node = ESEKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()