from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value= '0',
        description='Use dummy or not'
    )
    dummy = LaunchConfiguration('dummy')
    return LaunchDescription([
    
    	dummy_arg,
        Node(
            package='ia368_pkg',
            namespace='yolo_detector',
            executable='kinect_node',
            name='kinect_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='yolo_detector',
            executable='tf_node',
            name='tf_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='yolo_detector',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='ia368_pkg',
            namespace='yolo_detector',
            executable='dummy_creation_node',
            name='dummy_creation_node',
            output='screen',
            condition=IfCondition(dummy)
        )
    ])
