from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR Avoidance Node
        Node(
            package='mycar',
            executable='lidar_avoidance_node',
            name='lidar_safety',
            output='screen',
            parameters=[{
                # Set parameters directly here since we deleted the config folder
                'safe_distance': 0.5, # 50 cm
                'stop_distance': 0.3  # 30 cm
            }]
        )
    ])
