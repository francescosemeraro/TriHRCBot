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
     launch_arguments={'topic_to_subscribe':'/fake_body_tracking_data', 'to_save':'true', 'output':'screen'}.items())#,'namespace': LaunchConfiguration('namespace')
    
    ur_parameters_included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([robotic_system_launch_prefix, '/launch/ur_parameters.launch.py']))
    
    spawn_object_frames_included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([robotic_system_launch_prefix, '/launch/spawn_object_frames.launch.py']))

    azure_kinect_launch_prefix = get_package_share_directory('azure_kinect_ros_driver')

    azure_kinect_included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([azure_kinect_launch_prefix, '/launch/driver.launch.py']),
     launch_arguments={'color_resolution':'720P', 'fps':'30', 'body_tracking_enabled':'true', 'depth_mode':'NFOV_UNBINNED'}.items())
    
    classification_data_recorder= Node(
            package='robotic_system',
            namespace='user_study',
            executable='store_datapoint',
            name='store_classification_data',
            parameters=[
                {"bag_file_name": launch.substitutions.LaunchConfiguration('bag_file_name')},
                 {"selected_topic": "/classification_data"}],
            output="screen"
        )

    robot_performance_recorder= Node(
            package='robotic_system',
            namespace='user_study',
            executable='store_datapoint',
            name='store_robot_performance',
            parameters=[
                {"bag_file_name": launch.substitutions.LaunchConfiguration('bag_file_name')},
                 {"selected_topic": "/robot_performance"}],
            output="screen"
        )
    
    pose_calculator_node = Node(
            package='non_dyadic_ai',
            namespace='user_study',
            executable='full_pose_calculator',
            name='full_pose_calculator',
            parameters=[
                {"body_joints": True}],
            output="screen",
            remappings=[
            ('/user_study/utter_message', '/utter_message'),
            ('/user_study/mediated_pose', '/mediated_pose'),
            ('/user_study/converted_interaction_data', '/converted_interaction_data'),
            ('/user_study/visual_mediated_pose', '/visual_mediated_pose')
            ],
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
        #azure_kinect_included_launch,
        joint_activity_classifier_included_launch,
        ur_parameters_included_launch,
        spawn_object_frames_included_launch,
        classification_data_recorder,
        robot_performance_recorder,
        pose_calculator_node,
        sound_emitter
    ])
