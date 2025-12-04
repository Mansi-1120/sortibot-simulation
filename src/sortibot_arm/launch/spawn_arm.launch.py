from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('sortibot_arm'),
        'urdf',
        'sortibot_4dof.urdf'
    )

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

