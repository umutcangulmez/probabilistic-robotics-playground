from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to an empty world
    empty_world = '/opt/ros/humble/share/gazebo_ros/worlds/empty.world'


    return LaunchDescription([

        # Launch Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', empty_world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Launch Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen'
        ),

        # Leader robot node
        Node(
            package='follower_robot',
            executable='leader_node',
            name='leader_node',
            output='screen',
            emulate_tty=True
        ),

        # Follower robot node
        Node(
            package='follower_robot',
            executable='follower_node',
            name='follower_node',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='follower_robot',
            executable='measurement',
            name='measurement',
            output='screen',
            emulate_tty=True
        ),
    ])

