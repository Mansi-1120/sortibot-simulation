from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                "-s", "libgazebo_ros_factory.so",
                "/home/mansi/sortibot_ws/src/sortibot_gazebo/worlds/sortibot_world.world"
            ],
            output="screen"
        )
    ])


