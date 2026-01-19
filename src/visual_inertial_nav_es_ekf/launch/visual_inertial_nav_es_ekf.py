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

    ROBOT_NAME = 'turtlebot3_waffle'

    # Gazebo topics
    gz_tf_topic = f'/model/{ROBOT_NAME}/tf'
    gz_gt_odom_topic = f'/model/{ROBOT_NAME}/odometry'
    gz_imu_topic = '/imu'
    gz_camera_topic = '/camera/image'
    gz_cmd_vel_topic = '/cmd_vel'

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
        DeclareLaunchArgument('enable_smoother', default_value='false'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),

        # ================================================================
        # 2a) CRITICAL BRIDGE (Clock, TF, Odom, IMU, Cmd_vel)
        #     *Removed Camera from here to prevent clock lag*
        # ================================================================
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_core',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                f'{gz_gt_odom_topic}@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                f'{gz_tf_topic}@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'{gz_imu_topic}@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'{gz_cmd_vel_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[
                (gz_tf_topic, '/tf'),
                (gz_imu_topic, '/vehicle_blue/imu'),
            ],
            output='screen'
        ),

        # ================================================================
        # 2b) IMAGE BRIDGE (Dedicated for high bandwidth)
        #     *Uses ros_gz_image for optimized transport*
        # ================================================================
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='ros_gz_image_bridge',
            arguments=[gz_camera_topic],
            remappings=[
                (gz_camera_topic, '/vehicle_blue/camera/image')
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
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'world'],
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
        # 4) Keyboard Teleop
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            condition=IfCondition(enable_teleop),
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel_raw', gz_cmd_vel_topic),
            ],
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
        # 6) Visual Detector
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='visual_detector',
            name='visual_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmarks_file': landmarks_path,
                'known_landmark_height': 1.0,
                'camera_height': 0.20,
            }],
            output='screen'
        ),

        # ================================================================
        # 7) ES-EKF
        # ================================================================
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='es_ekf',
            name='es_ekf_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmarks_file': landmarks_path,
                'imu_topic': '/vehicle_blue/imu',
                'odom_frame_id': 'world',
                'base_frame_id': gt_frame,

                'imu_downsample_factor': 3,
                'filter_alpha': 0.35,

                'acc_enter_threshold': 0.15,
                'acc_exit_threshold': 0.30,
                'gyro_enter_threshold': 0.02,
                'gyro_exit_threshold': 0.05,
                'velocity_zupt_threshold': 0.05,
                'stationary_samples_required': 15,

                'sigma_a': 0.1,
                'sigma_w': 0.01,
                'sigma_ab': 1e-4,
                'sigma_wb': 1e-5,

                'R_range': 0.01,
                'R_bearing': 0.0025,
                'sigma_zupt': 0.02,

                'init_x': 0.0,
                'init_y': 0.0,
                'init_z': 0.0,

                'cam_offset_x': 0.12,
                'cam_offset_y': 0.0,
                'cam_offset_z': 0.20,
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