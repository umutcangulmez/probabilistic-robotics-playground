#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_package = 'visual_inertial_nav_es_ekf'

    return LaunchDescription([
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
            default_value='',
            description='Path to Gazebo world file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        # 2. Gazebo - ROS 2 Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                # Clock
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                # Ground Truth Odometry (For Validation)
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # TF (Internal model parts)
                '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # IMU
                '/vehicle_blue/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                # Camera
                '/vehicle_blue/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                # Command Velocity
                '/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[
                ('/model/vehicle_blue/tf', '/tf'),
            ],
            output='screen'
        ),
        # 3.1 Map -> World
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_world',
            arguments=['0','0','0',  '0','0','0','1',  'map', 'world'],
            output='screen',
        ),

        # 3.2 World -> Odom (The Missing Link!)
        # This connects the two TF trees together.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_world_to_odom',
            arguments=['0','0','0',  '0','0','0','1',  'world', 'vehicle_blue/odom'],
            output='screen',
        ),
        # Visual detector
        Node(
            package=ekf_package,
            executable='visual_detector',
            name='visual_detector',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ES-EKF
        Node(
            package=ekf_package,
            executable='es_ekf',
            name='es_ekf',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        # TF bridge (converts Gazebo poses to proper TF)
        Node(
            package='ekf_experiment',
            executable='gz_tf_bridge',
            name='gz_tf_bridge',
            output='screen'
        ),

        # 7. Trajectory Monitor
        Node(
            package='visual_inertial_nav_es_ekf',
            executable='trajectory_monitor',
            name='trajectory_monitor',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'world',
                'gt_frame': 'vehicle_blue/chassis',
                'ekf_topic': '/ekf_odom',
                'path_max_len': 5000,
            }],
            output='screen'
        ),
        # Experiment executor (delayed 5s)
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
                        'results_file': 'ekf_results.csv'
                    }]
                )
            ]
        ),
    ])