import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_DIR = os.path.join(os.getcwd(), 'src', 'turtle_experiments')

def generate_launch_description():
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # Burger doesnt' have the camera
    sdf_file = os.path.join(tb3_gazebo_pkg, 'models', 'turtlebot3_burger', 'model.sdf')
    # Waffle pi has the camera
    # sdf_file = os.path.join(tb3_gazebo_pkg, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')



    # Empty world generation
    #world_file = os.path.join(tb3_gazebo_pkg, 'worlds', 'empty_world.world')  # you can create your own

    # World with obstacles
    world_file = os.path.join(tb3_gazebo_pkg, 'worlds', 'turtlebot3_world.world')  # you can create your own

    gz = ExecuteProcess(cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'], output='screen')
    # Spawn SDF directly
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sdf_file,
                   '-entity', 'turtlebot3'],
        output='screen'
    )


    # robot_state_publisher: it's better to expand xacro before usage; this is minimal example
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
               output='screen', parameters=[{'robot_description': open("/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf").read()}])

    # rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
    #            output='screen', parameters=[{'robot_description': open("/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi.urdf").read()}])

    experiment = Node(package='turtle_experiments', executable='turtle_experiments', name='turtle_experiments', output='screen')

    return LaunchDescription([gz, spawn,rsp, experiment])
