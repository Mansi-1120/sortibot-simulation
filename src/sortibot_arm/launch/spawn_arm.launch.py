from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = "/home/mansi/sortibot_ws/src/sortibot_arm/urdf/sortibot_4dof.urdf"

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'sortibot_arm',
                '-file', urdf_file,
                '-x', '0.0',
                '-y', '-0.55',
                '-z', '0.55',
                '-Y', '1.57',
            ],
            output='screen'
        )
    ])

