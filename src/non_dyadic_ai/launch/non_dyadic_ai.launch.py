from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='non_dyadic_ai',
            namespace='torch_model',
            executable='torch_classifier',
            name='neural_network'
        ),
        Node(
            package='non_dyadic_ai',
            namespace='pose_optimiser',
            executable='pose_optimiser',
            name='pose_optimiser'
        )
    ])