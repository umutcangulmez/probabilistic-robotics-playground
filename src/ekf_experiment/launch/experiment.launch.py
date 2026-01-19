#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_package = 'visual_inertial_nav_es_ekf'

    # Get package share directory for file paths
    pkg_share = get_package_share_directory(ekf_package)

    # Paths to environment files
    landmarks_path = os.path.join(pkg_share, 'scripts', 'landmarks.json')
    world_path = os.path.join(pkg_share, 'scripts', 'project_world.sdf')

    return LaunchDescription([
        # ===================== Launch Arguments =====================
        DeclareLaunchArgument(
            'scenarios',
            default_value="['baseline']",
            description='List of scenarios to run'
        ),
        DeclareLaunchArgument(
            'num_runs',
            default_value='1',
            description='Number of runs per scenario'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=world_path,
            description='Path to Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='turtlebot3_waffle',
            description='Robot model: turtlebot3_waffle, turtlebot3_burger, or vehicle_blue'
        ),

        # ===================== Gazebo - ROS 2 Bridge =====================
        # Bridge for TurtleBot3 topics (remapped to legacy vehicle_blue names)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                # Clock
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                # Ground Truth Odometry (For Validation) - TurtleBot3
                '/model/turtlebot3_waffle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # TF (Internal model parts)
                '/model/turtlebot3_waffle/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # IMU - TurtleBot3 publishes on /imu
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                # Camera - TurtleBot3 publishes on /camera/image
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                # Command Velocity - TurtleBot3 listens on /cmd_vel
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # Odometry from diff drive
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            remappings=[
                # Remap TurtleBot3 topics to legacy vehicle_blue names
                ('/model/turtlebot3_waffle/tf', '/tf'),
                ('/imu', '/vehicle_blue/imu'),
                ('/camera/image', '/vehicle_blue/camera/image'),
                ('/cmd_vel', '/vehicle_blue/cmd_vel'),
                ('/model/turtlebot3_waffle/odometry', '/model/vehicle_blue/odometry'),
            ],
            output='screen'
        ),

        # ===================== TF Static Transforms =====================
        # Map -> World
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'world'],
            output='screen',
        ),

        # World -> Odom (connects the two TF trees)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'odom'],
            output='screen',
        ),

        # ===================== Visual Detector =====================
        Node(
            package=ekf_package,
            executable='visual_detector',
            name='visual_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmarks_file': landmarks_path,
                'known_landmark_height': 1.0,  # Must match generate_world.py --landmark-height
                'camera_height': 0.15,  # TurtleBot3 Waffle camera height
            }],
            output='screen'
        ),

        # ===================== ES-EKF =====================
        Node(
            package=ekf_package,
            executable='es_ekf',
            name='es_ekf',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmarks_file': landmarks_path,  # If EKF needs landmark positions
            }],
            output='screen'
        ),

        # ===================== TF Bridge =====================
        Node(
            package='ekf_experiment',
            executable='gz_tf_bridge',
            name='gz_tf_bridge',
            output='screen'
        ),

        # ===================== Trajectory Monitor =====================
        Node(
            package=ekf_package,
            executable='trajectory_monitor',
            name='trajectory_monitor',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'world',
                'gt_frame': 'base_footprint',  # Updated for TurtleBot3
                'ekf_topic': '/ekf_odom',
                'path_max_len': 5000,
            }],
            output='screen'
        ),

        # ===================== Experiment Executor (delayed 5s) =====================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ekf_experiment',
                    executable='experiment_executor',
                    name='experiment_executor',
                    output='screen',
                    parameters=[{
                        'scenarios': LaunchConfiguration('scenarios'),
                        'num_runs': LaunchConfiguration('num_runs'),
                        'results_file': 'ekf_results.csv',
                        'use_sim_time': True,
                    }]
                )
            ]
        ),
    ])