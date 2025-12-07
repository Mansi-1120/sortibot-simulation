from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sortibot_perception',
            executable='perception_node',
            name='sortibot_perception',
            output='screen'
        )
    ])
