
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path



def generate_launch_description():
    ld = LaunchDescription()
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'rviz', 'urdf_config.rviz')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_bringup'), 'launch/for_navigation.launch.py')
        )
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_bringup'), 'launch/for_slam.launch.py')
        )
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')]),
                    
)

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    ld.add_action(navigation_launch)
    ld.add_action(slam_launch)
    ld.add_action(gazebo)
    
    
    
    return LaunchDescription([
        ld,
        rviz2_node
    ])
