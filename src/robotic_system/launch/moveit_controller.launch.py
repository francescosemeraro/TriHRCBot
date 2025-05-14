import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    #ur_ros2_driver_launch_prefix = '/home/francesco/final_system_ws/install/ur_moveit_config/share/ur_moveit_config'
    ur_ros2_driver_launch_prefix = get_package_share_directory('ur_moveit_config')
    included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([ur_ros2_driver_launch_prefix, '/launch/ur_moveit.launch.py']),
     launch_arguments={'ur_type':'ur5e', 'launch_rviz':'true', 'rviz_config':'"default.rviz"','output':'screen'}.items())

    return LaunchDescription([
        included_launch,
        Node(
            package='robotic_system',
            namespace='moveit_controller',
            executable='moveit_controller',
            name='moveit_controller'
        )
    ])
