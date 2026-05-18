from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycar',
            executable='drive_node',
            name='driver'
        ),
        Node(
            package='mycar',
            executable='square_node',
            name='mission_logic'
        )
    ])
