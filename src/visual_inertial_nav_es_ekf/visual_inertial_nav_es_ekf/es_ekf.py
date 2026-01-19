#!/usr/bin/env python3
"""
ES-EKF - Diagnostic Version with Detailed Debug Output

This version removes complex features to help diagnose issues:
- NO cmd_vel gating (simpler stationarity detection)
- Standard full ZUPT (not velocity-only)
- NO stationary bias update
- Extensive debug output
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry


# =============================================================================
# Math utilities
# =============================================================================

def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0]
    ], dtype=float)


def normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-10 else np.array([0., 0., 0., 1.], dtype=float)


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
    angle = float(np.linalg.norm(rv))
    if angle < 1e-10:
        return np.array([rv[0]/2, rv[1]/2, rv[2]/2, 1.0], dtype=float)
    axis = rv / angle
    ha = angle / 2.0
    return np.array([axis[0]*np.sin(ha), axis[1]*np.sin(ha), axis[2]*np.sin(ha), np.cos(ha)], dtype=float)


def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


# =============================================================================
# ES-EKF core
# =============================================================================

class ESEKF:
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
        self.sigma_a = float(sigma_a)
        self.sigma_w = float(sigma_w)
        self.sigma_ab = float(sigma_ab)
        self.sigma_wb = float(sigma_wb)
        self.R_range = float(R_range)
        self.R_bearing = float(R_bearing)
        self.R_zupt = float(sigma_zupt) ** 2
        self.fixed_z = float(fixed_z)

        # Nominal state
        self.p = init_pos.copy() if init_pos is not None else np.zeros(3, dtype=float)
        self.v = np.zeros(3, dtype=float)
        self.q = np.array([0., 0., 0., 1.], dtype=float)
        self.ab = np.zeros(3, dtype=float)
        self.wb = np.zeros(3, dtype=float)

        # Gravity
        self.g_world = np.array([0., 0., -9.8], dtype=float)

        # Covariance
        self.P = np.diag([
            0.1, 0.1, 1e-4,
            0.01, 0.01, 1e-4,
            0.01, 0.01, 0.1,
            0.01, 0.01, 0.01,
            1e-4, 1e-4, 1e-4
        ]).astype(float)

        # Initialization
        self.is_initialized = False
        self.init_samples: List[dict] = []
        self.init_sample_count = 100

        # Stationarity
        self.is_stationary = False
        self.stationary_counter = 0

        # Debug counters
        self.predict_count = 0
        self.zupt_applied_count = 0
        self.visual_update_count = 0

    def add_init_sample(self, accel: np.ndarray, gyro: np.ndarray) -> bool:
        a_mag = float(np.linalg.norm(accel))
        g_mag = float(np.linalg.norm(gyro))
        if 8.0 < a_mag < 12.0 and g_mag < 0.1:
            self.init_samples.append({'accel': accel.copy(), 'gyro': gyro.copy()})
        if len(self.init_samples) >= self.init_sample_count:
            self._complete_init()
            return True
        return False

    def _complete_init(self):
        accels = np.array([s['accel'] for s in self.init_samples], dtype=float)
        gyros = np.array([s['gyro'] for s in self.init_samples], dtype=float)

        a_med = np.median(accels, axis=0)
        g_med = np.median(gyros, axis=0)

        measured_g = float(np.linalg.norm(a_med))
        self.g_world = np.array([0., 0., -measured_g], dtype=float)
        self.wb = g_med.copy()
        f_expected = np.array([0., 0., measured_g], dtype=float)
        self.ab = (a_med - f_expected).astype(float)

        self.is_initialized = True
        self.init_samples = []

    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float, zupt_config: dict):
        if not self.is_initialized or dt <= 0.0:
            return

        self.predict_count += 1

        # Bias correction
        f_body = accel - self.ab
        w_body = gyro - self.wb

        # Stationarity detection (simple IMU-only)
        self._update_stationarity(f_body, w_body, zupt_config)

        # State propagation
        rot = R.from_quat(self.q).as_matrix()
        a_world = rot @ f_body + self.g_world

        self.p = self.p + self.v * dt + 0.5 * a_world * (dt ** 2)
        self.v = self.v + a_world * dt

        w_norm = float(np.linalg.norm(w_body))
        if w_norm > 1e-10:
            dq = rotvec_to_quat(w_body * dt)
            self.q = normalize_quat(quat_mult(self.q, dq))

        self._apply_ground_constraints()
        self._propagate_covariance(f_body, w_body, rot, dt)

        # ZUPT when stationary
        if self.is_stationary:
            self._zupt_update()

    def _propagate_covariance(self, f_body, w_body, rot, dt):
        Fx = np.eye(15, dtype=float)
        Fx[0:3, 3:6] = np.eye(3) * dt
        Fx[3:6, 6:9] = -rot @ skew(f_body) * dt
        Fx[3:6, 9:12] = -rot * dt
        Fx[6:9, 6:9] = np.eye(3) - skew(w_body) * dt
        Fx[6:9, 12:15] = -np.eye(3) * dt

        sa2 = self.sigma_a ** 2
        sw2 = self.sigma_w ** 2
        sab2 = (self.sigma_ab ** 2) * dt
        swb2 = (self.sigma_wb ** 2) * dt

        Fi = np.zeros((15, 12), dtype=float)
        Fi[3:6, 0:3] = rot * dt
        Fi[6:9, 3:6] = np.eye(3) * dt
        Fi[9:12, 6:9] = np.eye(3)
        Fi[12:15, 9:12] = np.eye(3)

        Qc = np.diag([sa2]*3 + [sw2]*3 + [sab2]*3 + [swb2]*3)
        Q = Fi @ Qc @ Fi.T

        self.P = Fx @ self.P @ Fx.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def _apply_ground_constraints(self):
        self.p[2] = self.fixed_z
        self.v[2] = 0.0
        yaw = R.from_quat(self.q).as_euler('xyz')[2]
        self.q = normalize_quat(R.from_euler('z', yaw).as_quat())

        z_damp = 0.1
        for idx in [2, 5, 6, 7]:
            self.P[idx, :] *= z_damp
            self.P[:, idx] *= z_damp
            self.P[idx, idx] = max(self.P[idx, idx], 1e-12)

    def _update_stationarity(self, f_body, w_body, cfg):
        """Simple IMU-only stationarity detection."""
        rot = R.from_quat(self.q).as_matrix()
        g_body = rot.T @ self.g_world
        a_kin = f_body + g_body

        acc_xy = float(np.linalg.norm(a_kin[:2]))
        gyro_z = float(abs(w_body[2]))
        speed_xy = float(np.linalg.norm(self.v[:2]))

        acc_enter = float(cfg.get('acc_enter', 0.15))
        acc_exit = float(cfg.get('acc_exit', 0.30))
        gyro_enter = float(cfg.get('gyro_enter', 0.02))
        gyro_exit = float(cfg.get('gyro_exit', 0.05))
        vel_thresh = float(cfg.get('vel_thresh', 0.05))
        samples_req = int(cfg.get('samples_req', 15))

        if self.is_stationary:
            # Exit if any motion detected
            if acc_xy > acc_exit or gyro_z > gyro_exit or speed_xy > vel_thresh * 2:
                self.is_stationary = False
                self.stationary_counter = 0
        else:
            # Enter if all quiet
            is_quiet = (acc_xy < acc_enter and gyro_z < gyro_enter and speed_xy < vel_thresh)
            if is_quiet:
                self.stationary_counter += 1
                if self.stationary_counter >= samples_req:
                    self.is_stationary = True
            else:
                self.stationary_counter = 0

    def _zupt_update(self):
        """Standard full ZUPT (not velocity-only)."""
        y = -self.v[:2]

        H = np.zeros((2, 15), dtype=float)
        H[0, 3] = 1.0
        H[1, 4] = 1.0

        R_m = np.eye(2, dtype=float) * self.R_zupt

        dx = self._kalman_update(y, H, R_m)
        if dx is not None:
            self._inject_error(dx)
            self.zupt_applied_count += 1

    def update_range_bearing(self, lm_world: np.ndarray, z_range: float, z_bearing: float,
                             cam_offset: Optional[np.ndarray] = None):
        if not self.is_initialized:
            return
        if cam_offset is None:
            cam_offset = np.zeros(3, dtype=float)

        rot = R.from_quat(self.q).as_matrix()
        rot_T = rot.T

        lm_body = rot_T @ (lm_world - self.p)
        lm_cam = lm_body - cam_offset

        pred_range = float(np.linalg.norm(lm_cam[:2]))
        if pred_range < 0.5 or lm_cam[0] < 0.1:
            return

        pred_bearing = float(np.arctan2(lm_cam[1], lm_cam[0]))

        y = np.array([
            float(z_range) - pred_range,
            wrap_angle(float(z_bearing) - pred_bearing)
        ], dtype=float)

        x, y_lm = float(lm_cam[0]), float(lm_cam[1])
        r = pred_range

        dr_dl = np.array([x / r, y_lm / r, 0.0], dtype=float)
        db_dl = np.array([-y_lm / (r ** 2), x / (r ** 2), 0.0], dtype=float)

        H = np.zeros((2, 15), dtype=float)
        H[0, 0:3] = dr_dl @ (-rot_T)
        H[1, 0:3] = db_dl @ (-rot_T)
        H[0, 6:9] = dr_dl @ skew(lm_body)
        H[1, 6:9] = db_dl @ skew(lm_body)

        R_m = np.diag([self.R_range, self.R_bearing]).astype(float)

        dx = self._kalman_update(y, H, R_m)
        if dx is not None:
            self._inject_error(dx)
            self.visual_update_count += 1

    def _kalman_update(self, y, H, R_m):
        S = H @ self.P @ H.T + R_m
        try:
            K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
        except np.linalg.LinAlgError:
            return None

        dx = K @ y

        I_KH = np.eye(15, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_m @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return dx

    def _inject_error(self, dx):
        self.p += dx[0:3]
        self.v += dx[3:6]

        dtheta = dx[6:9]
        if float(np.linalg.norm(dtheta)) > 1e-10:
            dq = rotvec_to_quat(dtheta)
            self.q = normalize_quat(quat_mult(self.q, dq))

            G = np.eye(15, dtype=float)
            G[6:9, 6:9] = np.eye(3) - 0.5 * skew(dtheta)
            self.P = G @ self.P @ G.T
            self.P = 0.5 * (self.P + self.P.T)

        self.ab += dx[9:12]
        self.wb += dx[12:15]
        self.ab = np.clip(self.ab, -0.5, 0.5)
        self.wb = np.clip(self.wb, -0.1, 0.1)

        self._apply_ground_constraints()

    def get_pose_covariance_6x6(self):
        cov = np.zeros((6, 6), dtype=float)
        cov[0:3, 0:3] = self.P[0:3, 0:3]
        cov[3:6, 3:6] = self.P[6:9, 6:9]
        cov[0:3, 3:6] = self.P[0:3, 6:9]
        cov[3:6, 0:3] = self.P[6:9, 0:3]
        for i in [2, 3, 4]:
            cov[i, i] = max(cov[i, i], 1e-12)
        return cov


# =============================================================================
# ROS 2 Node
# =============================================================================
import json
import numpy as np


class ESEKFNode(Node):
    def __init__(self):
        super().__init__('es_ekf_node')

        # Parameters
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('vis_topic', '/visual_measurement')
        self.declare_parameter('odom_topic', '/ekf_odom')
        self.declare_parameter('path_topic', '/ekf_path')
        self.declare_parameter('odom_frame_id', 'world')
        self.declare_parameter('base_frame_id', 'base_footprint')

        self.declare_parameter('imu_downsample_factor', 3)
        self.declare_parameter('filter_alpha', 0.35)

        self.declare_parameter('acc_enter_threshold', 0.15)
        self.declare_parameter('acc_exit_threshold', 0.30)
        self.declare_parameter('gyro_enter_threshold', 0.02)
        self.declare_parameter('gyro_exit_threshold', 0.05)
        self.declare_parameter('velocity_zupt_threshold', 0.05)
        self.declare_parameter('stationary_samples_required', 15)

        self.declare_parameter('sigma_a', 0.1)
        self.declare_parameter('sigma_w', 0.01)
        self.declare_parameter('sigma_ab', 1e-4)
        self.declare_parameter('sigma_wb', 1e-5)

        self.declare_parameter('R_range', 0.01)
        self.declare_parameter('R_bearing', 0.0025)
        self.declare_parameter('sigma_zupt', 0.02)

        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 0.0)

        self.declare_parameter('cam_offset_x', 0.12)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.20)

        # Read parameters
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        imu_topic = self.get_parameter('imu_topic').value
        vis_topic = self.get_parameter('vis_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value


        self.downsample = max(1, int(self.get_parameter('imu_downsample_factor').value))
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

        # Create EKF
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

        # IMU buffers
        self.last_imu_time = None
        self.imu_buffer = []
        self.imu_accumulated_dt = 0.0
        self.accel_filtered = None
        self.gyro_filtered = None

        # Debug: track raw IMU values
        self.last_raw_accel = np.zeros(3)
        self.last_raw_gyro = np.zeros(3)
        self.last_a_kin = np.zeros(3)

        # ROS I/O
        self.sub_imu = self.create_subscription(Imu, imu_topic, self.imu_callback, 50)
        self.sub_vis = self.create_subscription(PointStamped, vis_topic, self.visual_callback, 50)

        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame_id

        self.create_timer(2.0, self.debug_callback)

        self.get_logger().info("=" * 60)
        self.get_logger().info("ES-EKF DIAGNOSTIC VERSION")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"IMU topic: {imu_topic}")
        self.get_logger().info(f"VIS topic: {vis_topic}")
        self.get_logger().info(f"Init pos: [{init_pos[0]:.2f}, {init_pos[1]:.2f}, {init_pos[2]:.2f}]")
        self.get_logger().info(f"Cam offset: [{self.cam_offset[0]:.2f}, {self.cam_offset[1]:.2f}]")
        self.get_logger().info(f"sigma_a={self.ekf.sigma_a}, sigma_w={self.ekf.sigma_w}")
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

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t

        if dt <= 0 or dt > 0.2:
            self.imu_buffer.clear()
            self.imu_accumulated_dt = 0.0
            self.accel_filtered = None
            self.gyro_filtered = None
            return

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

        # Initialization
        if not self.ekf.is_initialized:
            if self.ekf.add_init_sample(accel, gyro):
                self.get_logger().info("=" * 60)
                self.get_logger().info("ES-EKF INITIALIZED")
                self.get_logger().info(f"|g| = {np.linalg.norm(self.ekf.g_world):.4f}")
                self.get_logger().info(f"ab = [{self.ekf.ab[0]:.4f}, {self.ekf.ab[1]:.4f}, {self.ekf.ab[2]:.4f}]")
                self.get_logger().info(f"wb = [{self.ekf.wb[0]:.6f}, {self.ekf.wb[1]:.6f}, {self.ekf.wb[2]:.6f}]")
                self.get_logger().info("=" * 60)
            return

        # Low-pass filter
        if self.accel_filtered is None:
            self.accel_filtered = accel.copy()
            self.gyro_filtered = gyro.copy()
        else:
            self.accel_filtered = self.alpha * accel + (1.0 - self.alpha) * self.accel_filtered
            self.gyro_filtered = self.alpha * gyro + (1.0 - self.alpha) * self.gyro_filtered

        # Downsample
        self.imu_buffer.append({'accel': self.accel_filtered.copy(), 'gyro': self.gyro_filtered.copy()})
        self.imu_accumulated_dt += dt

        if len(self.imu_buffer) < self.downsample:
            return

        avg_accel = np.mean([s['accel'] for s in self.imu_buffer], axis=0)
        avg_gyro = np.mean([s['gyro'] for s in self.imu_buffer], axis=0)
        batch_dt = float(self.imu_accumulated_dt)

        self.imu_buffer.clear()
        self.imu_accumulated_dt = 0.0

        # Compute kinematic acceleration for debug
        f_body = avg_accel - self.ekf.ab
        rot = R.from_quat(self.ekf.q).as_matrix()
        g_body = rot.T @ self.ekf.g_world
        self.last_a_kin = f_body + g_body

        # Predict
        self.ekf.predict(avg_accel, avg_gyro, batch_dt, self.zupt_config)

        self.publish_state(msg.header.stamp)

    # def visual_callback(self, msg: PointStamped):
    #     if not self.ekf.is_initialized:
    #         return
    #     lm_id = int(msg.point.z)
    #     if lm_id not in self.landmarks:
    #         return
    #
    #     z_bearing = float(msg.point.x)
    #     z_range = float(msg.point.y)
    #     lm_world = self.landmarks[lm_id]
    #
    #     # Debug: compute expected vs measured
    #     rot = R.from_quat(self.ekf.q).as_matrix()
    #     lm_body = rot.T @ (lm_world - self.ekf.p)
    #     lm_cam = lm_body - self.cam_offset
    #     pred_range = float(np.linalg.norm(lm_cam[:2]))
    #     pred_bearing = float(np.arctan2(lm_cam[1], lm_cam[0]))
    #
    #     range_err = z_range - pred_range
    #     bearing_err = wrap_angle(z_bearing - pred_bearing)
    #
    #     self.ekf.update_range_bearing(lm_world, z_range, z_bearing, self.cam_offset)
    def visual_callback(self, msg: PointStamped):
        if not self.ekf.is_initialized:
            return

    # Check time delay
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        delay = (now - msg_time).nanoseconds / 1e9
        self.get_logger().info(f"Visual measurement delay: {delay:.3f}s", throttle_duration_sec=1.0)

        lm_id = int(msg.point.z)
        if lm_id not in self.landmarks:
            self.get_logger().warn(f"Unknown landmark ID: {lm_id}")
            return

        z_bearing = float(msg.point.x)
        z_range = float(msg.point.y)
        lm_world = self.landmarks[lm_id]

        # Debug: compute expected vs measured
        rot = R.from_quat(self.ekf.q).as_matrix()
        lm_body = rot.T @ (lm_world - self.ekf.p)
        lm_cam = lm_body - self.cam_offset
        pred_range = float(np.linalg.norm(lm_cam[:2]))
        pred_bearing = float(np.arctan2(lm_cam[1], lm_cam[0]))

        range_err = z_range - pred_range
        bearing_err = wrap_angle(z_bearing - pred_bearing)

        self.get_logger().info(
            f"LM{lm_id}: meas=[r={z_range:.2f}, b={np.degrees(z_bearing):.1f}°] "
            f"pred=[r={pred_range:.2f}, b={np.degrees(pred_bearing):.1f}°] "
            f"err=[dr={range_err:.2f}, db={np.degrees(bearing_err):.1f}°]"
        )

        self.ekf.update_range_bearing(lm_world, z_range, z_bearing, self.cam_offset)
    def publish_state(self, stamp):
        ekf = self.ekf

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

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)
        self.path_msg.header.stamp = stamp
        self.pub_path.publish(self.path_msg)

    def debug_callback(self):
        if not self.ekf.is_initialized:
            self.get_logger().info("Waiting for initialization...")
            return

        ekf = self.ekf
        yaw_deg = R.from_quat(ekf.q).as_euler('xyz', degrees=True)[2]
        speed = float(np.linalg.norm(ekf.v[:2]))

        stat = "STAT" if ekf.is_stationary else "MOVE"

        # Compute a_kin_xy for debug
        a_kin_xy = float(np.linalg.norm(self.last_a_kin[:2]))
        gyro_z = float(abs(self.last_raw_gyro[2] - ekf.wb[2]))

        self.get_logger().info("-" * 70)
        self.get_logger().info(f"POS: [{ekf.p[0]:7.3f}, {ekf.p[1]:7.3f}] m")
        self.get_logger().info(f"VEL: [{ekf.v[0]:7.3f}, {ekf.v[1]:7.3f}] m/s  |v|={speed:.3f}")
        self.get_logger().info(f"YAW: {yaw_deg:7.1f}°  [{stat}]")
        self.get_logger().info(f"a_kin_xy={a_kin_xy:.3f} m/s²  gyro_z={gyro_z:.4f} rad/s")
        self.get_logger().info(f"ab=[{ekf.ab[0]:.4f},{ekf.ab[1]:.4f},{ekf.ab[2]:.4f}]")
        self.get_logger().info(f"Predictions: {ekf.predict_count}, ZUPT: {ekf.zupt_applied_count}, Visual: {ekf.visual_update_count}")
        self.get_logger().info(f"tr(P)={np.trace(ekf.P):.4f}, P_pos=[{ekf.P[0,0]:.4f},{ekf.P[1,1]:.4f}]")

def main():
    rclpy.init()
    node = ESEKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()