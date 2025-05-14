import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    azure_kinect_launch_prefix = get_package_share_directory('azure_kinect_ros_driver')

    included_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([azure_kinect_launch_prefix, '/launch/driver.launch.py']),
     launch_arguments={'color_resolution':'720P', 'fps':'30', 'body_tracking_enabled':'true', 'depth_mode':'NFOV_UNBINNED'}.items())

    included_launch2 = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(['/home/francesco/final_system_ws/src/robotic_system/launch/spawn_object_frames.launch.py']))
    
    included_launch3 = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(['/home/francesco/final_system_ws/src/non_dyadic_ai/launch/non_dyadic_ai.launch.py']))

    bag_file_name = DeclareLaunchArgument(
        'bag_file_name', 
        default_value=TextSubstitution(text='tag'), 
        description="Tag related to the acquisition")
    
    condition = DeclareLaunchArgument(
        'condition', 
        default_value=TextSubstitution(text='not set'), 
        description="Name of the condition being administered in the user study")
    
    datapoint_recorder= Node(
            package='robotic_system',
            namespace='user_study',
            executable='store_datapoint',
            name='store_datapoint',
            parameters=[
                {"bag_file_name": launch.substitutions.LaunchConfiguration('bag_file_name')}]
        )
    
    robot_controller= Node(
            package='robotic_system',
            namespace='user_study',
            executable='test_transitions',
            name='robot_controller',
            parameters=[
                {"condition": launch.substitutions.LaunchConfiguration('condition')}]
        )
    
    sound_emitter= Node(
            package='robotic_system',
            namespace='user_study',
            executable='produce_sound',
            name='sound_emitter',
        )

    return LaunchDescription([
        included_launch,
        included_launch2,
        bag_file_name,
        condition,
        datapoint_recorder,
        robot_controller,
        sound_emitter,
        ])

