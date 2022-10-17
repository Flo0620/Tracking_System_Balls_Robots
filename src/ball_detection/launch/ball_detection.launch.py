from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_detection',
            namespace='yoloWrapper',
            executable='YoloWrapper',
            name='yoloWrapper'
        )
    ])
