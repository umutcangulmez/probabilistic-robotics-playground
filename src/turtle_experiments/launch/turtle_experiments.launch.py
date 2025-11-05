from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    empty_world = '/opt/ros/humble/share/gazebo_ros/worlds/empty.world'


    return LaunchDescription([

        # Launch Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', empty_world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Launch Gazebo client
        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen'
        ),

        # Follower robot node
        Node(
            package='turtle_experiments',
            executable='turtle_experiments',
            name='turtle_experiments',
            output='screen',
            emulate_tty=True
        ),
    ])

