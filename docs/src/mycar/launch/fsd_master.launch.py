import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    yolo_launch_path = os.path.join(
        get_package_share_directory('yolov5_ros2'),
        'launch',
        'yolov5_ros2.launch.py'
    )

    return LaunchDescription([
        # 1. Start YOLOv5 detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch_path),
            launch_arguments={'model': 'traffic_signs_640s_7_0'}.items()
        ),

        # 2. FSD Nodes
        Node(package='mycar', executable='fsd_lane_keep', name='fsd_lane', output='screen'),
        Node(package='mycar', executable='fsd_lidar_safety', name='fsd_safety', output='screen'),
        Node(package='mycar', executable='fsd_mission_manager', name='fsd_manager', output='screen'),
        Node(package='mycar', executable='yolo_logic_node', name='yolo_logic', output='screen'),
    ])

