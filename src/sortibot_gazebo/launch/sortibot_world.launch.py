from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('sortibot_gazebo'),
        'worlds',
        'sortibot_world.world'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                "-s", "libgazebo_ros_factory.so",
                world_path
            ],
            output="screen"
        )
    ])

