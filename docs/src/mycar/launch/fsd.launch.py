import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to configuration file
    config_path = os.path.join(
        get_package_share_directory('mycar'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        # 1. Camera Setup
        Node(
            package='mycar',
            executable='camera_tilt_node',
            name='camera_setup',
            output='screen',
            parameters=[config_path]
        ),
        
        # 2. Lane Detection
        Node(
            package='mycar',
            executable='lane_detect_node',
            name='lane_node',
            output='screen',
            parameters=[config_path]
        ),
        
        # 3. YOLO Logic
        Node(
            package='mycar',
            executable='yolo_logic_node',
            name='yolo_logic',
            output='screen',
            parameters=[config_path]
        ),
        
        # 4. Lidar Avoidance
        Node(
            package='mycar',
            executable='lidar_avoidance_node',
            name='lidar_safety',
            output='screen',
            parameters=[config_path]
        ),
        
        # 5. Mission Manager (Arbiter)
        Node(
            package='mycar',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen',
            parameters=[config_path]
        )
    ])
