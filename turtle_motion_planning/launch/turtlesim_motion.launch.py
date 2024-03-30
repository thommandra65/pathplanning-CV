from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_motion_planning',  # Replace with your package name
            executable='turtlesim_motion_pose',  # Replace with your executable name
            name='turtlesim_motion_node',
            output='screen',
        )
    ])

