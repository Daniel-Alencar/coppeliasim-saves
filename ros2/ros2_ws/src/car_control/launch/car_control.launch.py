from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            namespace='car_control',
            executable='car_control',
            output='screen'
        ),
    ])
