import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    
    topic_to_subscribe = DeclareLaunchArgument(
        'topic_to_subscribe', 
        default_value=TextSubstitution(text='/raw_datapoint'), 
        description="Name of the topic to subscribe for tf_classifier to get skeleton poses")
    
    to_save = DeclareLaunchArgument(
        'to_save', 
        default_value='false', 
        description="Enabler of the publisher for bag file saving")
    
    tf_classifier= Node(
            package='non_dyadic_ai',
            #namespace='final_user_study',
            executable='tf_classifier',
            name='tf_classifier',
            parameters=[
                {"topic_to_subscribe": launch.substitutions.LaunchConfiguration('topic_to_subscribe')},
                {"to_save": launch.substitutions.LaunchConfiguration('to_save')}]
        )
    
    interaction_data_converter= Node(
            package='robotic_system',
            #namespace='final_user_study',
            executable='interaction_data_converter',
            name='interaction_data_converter',
        )
    

    return LaunchDescription([
        topic_to_subscribe,
        to_save,
        interaction_data_converter,
        tf_classifier
        ])

