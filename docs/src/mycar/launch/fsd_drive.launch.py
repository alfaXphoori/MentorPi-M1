import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Lane Keeping Node (Eye)
        Node(
            package='mycar',
            executable='lane_keep_node',
            name='lane_keep',
            output='screen'
        ),
        
        # 2. Lidar Avoidance Node (Safety/Dodge - Priority High)
        Node(
            package='mycar',
            executable='lidar_avoidance_node',
            name='lidar_avoid',
            output='screen'
        ),
        
        # 3. Mission Manager (Brain)
        Node(
            package='mycar',
            executable='mission_manager',
            name='mission_manager',
            output='screen'
        )
    ])
