
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'rviz', 'new_navigation.rviz')
       # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')]),
)
                                 
                                    
                                    
    bringup_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_bringup'), 'launch/custom_bringup_launch.py')
        )
    )                         



    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    ld.add_action(bringup_nav)
    ld.add_action(gazebo)
    
    
    
    
    
   
    return LaunchDescription([
        ld,
        rviz2_node
    ])
