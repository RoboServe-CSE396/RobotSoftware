
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node


def generate_launch_description():
 
    teleop_node = Node(
        package='my_robot_bringup',
        executable='custom_teleop',
        name='custom_teleop'
    )
    
   
    
    
    
   
    return LaunchDescription([
        teleop_node
    ])
