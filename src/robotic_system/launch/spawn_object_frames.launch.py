import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():      
    
    link_camera_to_tf2_tree= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            #arguments=['0.46', '0.32', '0.8', '-0.2078277', '-0.086085', '0.9002006', '-0.3728753', 'base_link', 'camera_base']
            arguments=['0.56', '0.34', '0.8', '-0.184192', '-0.0762948', '0.9053324', '-0.3750009', 'base_link', 'camera_base'] #before ASK
            #arguments=['0.56', '0.325', '0.815', '-0.1125926', '-0.0466374', '0.9169931', '-0.379831', 'base_link', 'camera_base'] #ASK
    )
    spawn_target_object= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_target_object',
            arguments=['0.0', '0.0', '0.235', '0.0', '0.0', '0.0', 'tool0', 'target_object']
            #arguments=['0.0', '0.0', '0.17', '0.0', '0.0', '0.0', 'tool0', 'target_object']
    )
    spawn_shapes_face= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_shapes_face',
            #arguments=['0.0', '0.075', '0.065', '1.57', '0.0', '0.0',  'target_object', 'shapes_face']
            arguments=['0.0', '0.075', '0.0', '1.57', '0.0', '0.0',  'target_object', 'shapes_face']
    )
    spawn_gears_face= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_gears_face',
            #arguments=['0.0', '-0.075', '0.065', '-1.57', '0.0', '0.0', 'target_object', 'gears_face']
            arguments=['0.0', '-0.075', '0.0', '-1.57', '0.0', '0.0', 'target_object', 'gears_face']
    )
    spawn_beads_face= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_beads_face',
            arguments=['0.0', '0.0', '0.075', '1.57', '6.018', '0.0', 'target_object', 'beads_face']
            #arguments=['0.0', '0.0', '0.14', '1.57', '6.018', '0.0',  'target_object', 'beads_face']
            #arguments=['0.0', '0.0', '0.14', '1.57', '5.495', '0.0',  'target_object', 'beads_face']
            #arguments=['0.0', '0.0', '0.075', '0.0', '0.0', '0.785',  'target_object', 'beads_face']
    )
    spawn_tiles_face= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_tiles_face',
            #arguments=['0.075', '0.0', '0.065', '0.0', '0.0  ', '0.0', 'target_object', 'tiles_face']
            arguments=['0.075', '0.0', '0.0', '0.0', '0.0', '0.0', 'target_object', 'tiles_face']
    )
    base_to_depth= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera_depth_link',
            arguments=['0.0', '0.0', '0.002', '0.525', '-0.525', '0.473', '-0.473', 'camera_base', 'depth_camera_link']
    )
    
    return LaunchDescription([
        base_to_depth, #Only for testing
        link_camera_to_tf2_tree,
        spawn_target_object,
        spawn_beads_face,
        spawn_gears_face,
        spawn_shapes_face,
        spawn_tiles_face
    ])

