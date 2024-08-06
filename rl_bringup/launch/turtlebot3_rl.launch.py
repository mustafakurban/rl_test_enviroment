import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'my_custom_world.world'
    world = os.path.join(get_package_share_directory('my_turtlebot3_rl'),
                         'worlds', world_file_name)

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='SDF world file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/turtlebot3_world.launch.py']),
            launch_arguments={'world': world}.items(),
        ),
    ])
