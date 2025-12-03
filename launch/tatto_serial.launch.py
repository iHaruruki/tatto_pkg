import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='tatto_pkg',
            executable='tatto_serial_node',
            name='tatto_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1', # Please check USB port
            }]
        ),
        
    ])