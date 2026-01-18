#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenarios',
            default_value="['baseline']",
            description='List of scenarios: baseline, high_imu_noise, high_visual_noise, fast_motion'
        ),
        DeclareLaunchArgument(
            'num_runs',
            default_value='1',
            description='Number of runs per scenario'
        ),
        DeclareLaunchArgument(
            'results_file',
            default_value='ekf_results.csv',
            description='Output CSV filename'
        ),

        Node(
            package='ekf_experiment',
            executable='experiment_executor',
            name='experiment_executor',
            output='screen',
            parameters=[{
                'scenarios': LaunchConfiguration('scenarios'),
                'num_runs': LaunchConfiguration('num_runs'),
                'results_file': LaunchConfiguration('results_file')
            }]
        ),
    ])