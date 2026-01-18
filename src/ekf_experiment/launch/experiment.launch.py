#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

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


        # ROS-Gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/vehicle_blue/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/world/project_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            output='screen'
        ),

        # Visual detector
        Node(
            package=ekf_package,
            executable='visual_detector',
            name='visual_detector',
            output='screen'
        ),

        # ES-EKF
        Node(
            package=ekf_package,
            executable='es_ekf',
            name='es_ekf',
            output='screen'
        ),
        # TF bridge (converts Gazebo poses to proper TF)
        Node(
            package='ekf_experiment',
            executable='gz_tf_bridge',
            name='gz_tf_bridge',
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