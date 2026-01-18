"""
Range + Bearing Visual Detector (Camera-based)
Detects colored landmarks from camera images and publishes range/bearing measurements.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class VisualDetectorCamera(Node):
    def __init__(self):
        super().__init__('visual_detector_camera')

        # Parameters
        self.declare_parameter('fov', 1.0472)  # ~60 degrees
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('camera_offset_x', 1.0)
        self.declare_parameter('bearing_stddev', 0.05)
        self.declare_parameter('range_stddev', 0.1)
        self.declare_parameter('camera_topic', '/vehicle_blue/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/vehicle_blue/camera/camera_info')

        self.fov = self.get_parameter('fov').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.bearing_stddev = self.get_parameter('bearing_stddev').value
        self.range_stddev = self.get_parameter('range_stddev').value

        # Landmark definitions with HSV color ranges
        # Format: {id: {'name': str, 'hsv_lower': array, 'hsv_upper': array, 'real_size': float}}
        self.landmarks = {
            1: {
                'name': 'Red',
                'hsv_lower': np.array([0, 120, 70]),
                'hsv_upper': np.array([10, 255, 255]),
                'hsv_lower2': np.array([170, 120, 70]),  # Red wraps around
                'hsv_upper2': np.array([180, 255, 255]),
                'real_size': 0.3  # Real-world diameter in meters
            },
            2: {
                'name': 'Green',
                'hsv_lower': np.array([40, 80, 80]),
                'hsv_upper': np.array([80, 255, 255]),
                'real_size': 0.3
            },
            3: {
                'name': 'Blue',
                'hsv_lower': np.array([100, 120, 70]),
                'hsv_upper': np.array([130, 255, 255]),
                'real_size': 0.3
            },
            4: {
                'name': 'Yellow',
                'hsv_lower': np.array([20, 100, 100]),
                'hsv_upper': np.array([35, 255, 255]),
                'real_size': 0.3
            }
        }

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = None
        self.image_height = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(PointStamped, '/visual_measurement', 10)

        # Debug publisher (optional - shows detection visualization)
        self.debug_pub = self.create_publisher(Image, '/visual_detector/debug_image', 10)

        self.get_logger().info('Visual detector initialized, waiting for camera data...')

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics from CameraInfo message."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(f'Camera info received: {self.image_width}x{self.image_height}')
            self.get_logger().info(f'Focal length: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}')

    def image_callback(self, msg: Image):
        """Process incoming camera image to detect landmarks."""
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Convert to HSV for color detection
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        debug_image = cv_image.copy() if self.debug_pub.get_subscription_count() > 0 else None

        for lm_id, data in self.landmarks.items():
            detection = self.detect_landmark(hsv_image, data, debug_image)

            if detection is not None:
                cx, cy, pixel_diameter = detection

                # Calculate bearing from pixel position
                bearing = self.pixel_to_bearing(cx)

                # Estimate range from apparent size
                dist = self.estimate_range(pixel_diameter, data['real_size'])

                # Check if within valid detection range and FOV
                if self.min_range < dist < self.max_range and abs(bearing) < (self.fov / 2.0):
                    # Add measurement noise
                    meas_bearing = bearing + np.random.normal(0, self.bearing_stddev)
                    meas_range = dist + np.random.normal(0, self.range_stddev)
                    meas_range = max(0.1, meas_range)

                    # Publish measurement
                    meas_msg = PointStamped()
                    meas_msg.header.stamp = msg.header.stamp
                    meas_msg.header.frame_id = 'vehicle_blue/camera_frame'
                    meas_msg.point.x = float(meas_bearing)
                    meas_msg.point.y = float(meas_range)
                    meas_msg.point.z = float(lm_id)

                    self.pub.publish(meas_msg)

                    if debug_image is not None:
                        cv2.putText(debug_image, f'{data["name"]}: r={meas_range:.1f}m, b={math.degrees(meas_bearing):.1f}deg',
                                    (int(cx) - 50, int(cy) - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Publish debug image
        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Debug image publish error: {e}')

    def detect_landmark(self, hsv_image, landmark_data, debug_image=None):
        """
        Detect a colored landmark in the HSV image.
        Returns (center_x, center_y, diameter) or None if not detected.
        """
        # Create color mask
        mask = cv2.inRange(hsv_image, landmark_data['hsv_lower'], landmark_data['hsv_upper'])

        # Handle red color wrap-around
        if 'hsv_lower2' in landmark_data:
            mask2 = cv2.inRange(hsv_image, landmark_data['hsv_lower2'], landmark_data['hsv_upper2'])
            mask = cv2.bitwise_or(mask, mask2)

        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find the largest contour (assuming it's the landmark)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Minimum area threshold to filter noise
        if area < 100:
            return None

        # Get bounding circle
        (cx, cy), radius = cv2.minEnclosingCircle(largest_contour)
        diameter = radius * 2

        # Draw on debug image
        if debug_image is not None:
            cv2.circle(debug_image, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(debug_image, (int(cx), int(cy)), 3, (0, 0, 255), -1)
            cv2.putText(debug_image, landmark_data['name'], (int(cx) - 20, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return cx, cy, diameter

    def pixel_to_bearing(self, pixel_x):
        """
        Convert pixel x-coordinate to bearing angle.
        Positive bearing = left of center (standard convention).
        """
        # Principal point (optical center)
        cx = self.camera_matrix[0, 2]
        fx = self.camera_matrix[0, 0]

        # Pixel offset from center (positive = right in image)
        dx = pixel_x - cx

        # Bearing angle (negative because right in image = negative bearing)
        bearing = -math.atan2(dx, fx)

        return bearing

    def estimate_range(self, pixel_diameter, real_diameter):
        """
        Estimate range to object using pinhole camera model.
        range = (real_size * focal_length) / pixel_size
        """
        if pixel_diameter <= 0:
            return float('inf')

        # Use average focal length
        f = (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2.0

        # Pinhole camera equation: pixel_size / f = real_size / range
        # Therefore: range = real_size * f / pixel_size
        estimated_range = (real_diameter * f) / pixel_diameter

        return estimated_range


def main(args=None):
    rclpy.init(args=args)
    node = VisualDetectorCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    ### change landmark density noise same
    ### landmark fix change noise
    ### prediction only, measurement only
    ### landmark type, different measurements
