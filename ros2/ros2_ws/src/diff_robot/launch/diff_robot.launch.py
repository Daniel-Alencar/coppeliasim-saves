from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value='0',
        description='Use dummy or not'
    )
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='/myRobot',
        description='Path of the differential robot in the CoppeliaSim scene'
    )
    dummy = LaunchConfiguration('dummy')
    robot = LaunchConfiguration('robot')
    return LaunchDescription([
        dummy_arg,
        robot_arg,
        Node(
            package='diff_robot',
            namespace='diff_robot',
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            parameters=[{'robot': robot}]
        ),
        Node(
            package='diff_robot',   
            namespace='diff_robot',
            executable='dummy_driver',
            name='dummy_driver',
            output='screen',
            condition=IfCondition(dummy)
        ),
    ])
