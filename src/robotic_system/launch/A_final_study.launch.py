import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    #ur_ros2_driver_launch_prefix = '/home/francesco/final_system_ws/install/ur_moveit_config/share/ur_moveit_config'
    robotic_system_launch_prefix = get_package_share_directory('robotic_system')

    bag_file_name = DeclareLaunchArgument(
        'bag_file_name', 
        default_value=TextSubstitution(text='tag'), 
        description="Tag related to the acquisition")
    
    joint_activity_classifier_included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([robotic_system_launch_prefix, '/launch/joint_activity_classifier.launch.py']),
     launch_arguments={'topic_to_subscribe':'/body_tracking_data', 'to_save':'true', 'output':'screen'}.items())
    
    spawn_object_frames_included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([robotic_system_launch_prefix, '/launch/spawn_object_frames.launch.py']))

    classification_data_recorder= Node(
            package='robotic_system',
            namespace='user_study',
            executable='store_datapoint',
            name='store_classification_data',
            parameters=[
                {"bag_file_name": launch.substitutions.LaunchConfiguration('bag_file_name')},
                 {"selected_topic": "/classification_data"}],
            output = "screen"
        )

    pose_calculator_node = Node(
            package='robotic_system',
            namespace='', #don't write anything here
            executable='no_pose_calculator',
            name='no_pose_calculator',
            output="screen"
        )
    
    sound_emitter= Node(
            package='robotic_system',
            namespace='user_study',
            executable='produce_sound',
            name='sound_emitter',
            output="screen"
        )
    
    return LaunchDescription([
        bag_file_name,
        classification_data_recorder,
        joint_activity_classifier_included_launch,
        spawn_object_frames_included_launch,
        pose_calculator_node,
        sound_emitter
    ])
