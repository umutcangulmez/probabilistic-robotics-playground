from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_smoother = LaunchConfiguration('enable_smoother')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_visual = LaunchConfiguration('enable_visual')

    ROBOT_NAME = 'turtlebot3_waffle'

    gz_tf_topic = f'/model/{ROBOT_NAME}/tf'
    gz_gt_odom_topic = f'/odom'
    gz_imu_topic = '/imu'
    gz_camera_topic = '/camera/image'
    gz_cmd_vel_topic = f'/cmd_vel'

    # TF frames
    gt_frame = f'{ROBOT_NAME}/base_footprint'
    odom_frame = 'odom'

    pkg_share = get_package_share_directory('visual_inertial_nav_es_ekf')
    landmarks_path = os.path.join(pkg_share, 'scripts', 'landmarks.json')

    return LaunchDescription([
        # ================================================================
        # 1) Launch Arguments
        # ================================================================
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_smoother', default_value='true'),
        DeclareLaunchArgument('enable_teleop', default_value='false'),
        DeclareLaunchArgument('enable_visual', default_value='false'),

        # ================================================================
        # 2) Gazebo <-> ROS 2 Bridge
        # ================================================================
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                f'{gz_gt_odom_topic}@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                f'{gz_tf_topic}@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'{gz_imu_topic}@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'{gz_camera_topic}@sensor_msgs/msg/Image@gz.msgs.Image',
                f'{gz_cmd_vel_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[
                (gz_tf_topic, '/tf'),
            ],
            output='screen'
        ),

        # ================================================================
        # 3) Static Transforms
        # ================================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', odom_frame],
            output='screen',
        ),

        # ================================================================
        # 4) Keyboard Teleop (optional)
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            condition=IfCondition(enable_teleop),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ================================================================
        # 5) Velocity Smoother
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='velocity_smoother',
            name='velocity_smoother',
            condition=IfCondition(enable_smoother),
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/cmd_vel_raw',
                'output_topic': gz_cmd_vel_topic,
                'max_linear': 1.0,
                'max_angular': 0.5,
                'max_linear_accel': 0.2,
                'max_linear_decel': 1.0,
                'max_angular_accel': 0.2,
                'max_angular_decel': 0.2,
                'cmd_timeout': 0.5,
            }],
            output='screen'
        ),

        # ================================================================
        # 6) Visual Detector (Camera-Based)
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='visual_detector',
            name='visual_detector',
            condition=IfCondition(enable_visual),
            parameters=[{
                'use_sim_time': use_sim_time,
                'mode': 'color',  # 'color' or 'aruco'
                'camera_topic': gz_camera_topic,  # /camera from SDF
                'fov': 1.085,
                'image_width': 1280,  # From SDF
                'image_height': 720,  # From SDF
                'noise_bearing': 0.0,  # rad
                'noise_range': 0.0,    # m
                'known_landmark_height': 1.0,  # 1.0m cylinders from SDF
                'known_landmark_diameter': 0.4,  # 0.2m radius = 0.4m diameter
                'camera_height': 0.103,  # 0.010 (base) + 0.093 (camera_link z)
                'landmarks_file': landmarks_path,   # Empty = use defaults matching SDF
                'min_contour_area': 100,
                'max_range': 20.0,
                'min_range': 0.5,
            }],
            output='screen'
        ),

        # ================================================================
        # 7) ES-EKF (Improved Version - No Outlier Rejection)
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='es_ekf',  # Update entry point in setup.py
            name='es_ekf_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'imu_topic': gz_imu_topic,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'landmarks_file': landmarks_path,

                # IMU processing
                'filter_alpha': 0.35,

                # Stationarity detection
                'acc_enter_threshold': 0.15,
                'acc_exit_threshold': 0.30,
                'gyro_enter_threshold': 0.02,
                'gyro_exit_threshold': 0.05,
                'velocity_zupt_threshold': 0.05,
                'stationary_samples_required': 15,

                # Process noise - increased for better motion tracking
                # These are BASE values; they get scaled up during motion
                'sigma_a': 0.15,    # accelerometer noise [m/s^2]
                'sigma_w': 0.02,    # gyroscope noise [rad/s]
                'sigma_ab': 1e-4,   # accel bias drift
                'sigma_wb': 1e-5,   # gyro bias drift

                # Measurement noise
                'R_range': 0.25,    # range variance [m^2] (0.05m std)
                'R_bearing': 0.01,   # bearing variance [rad^2] (~0.03rad std)
                'sigma_zupt': 0.02,

                # Initial position (robot starts at origin)
                'init_x': 0.0,
                'init_y': 0.0,
                'init_z': 0.0,

                # Camera offset (must match visual_detector)
                'cam_offset_x': 0.10,
                'cam_offset_y': 0.0,
                'cam_offset_z': 0.0,
            }],
            output='screen'
        ),

        # ================================================================
        # 8) Trajectory Monitor
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='trajectory_monitor',
            name='trajectory_monitor',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': odom_frame,
                'gt_frame': gt_frame,
                'ekf_topic': '/ekf_odom',
                'path_max_len': 5000,
                'nees_window': 100,
            }],
            output='screen'
        ),
    ])