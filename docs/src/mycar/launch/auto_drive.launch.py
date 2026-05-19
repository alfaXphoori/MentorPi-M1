import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # YOLOv5 Launch File Path
    yolo_launch_path = os.path.join(
        get_package_share_directory('yolov5_ros2'),
        'launch',
        'yolov5_ros2.launch.py'
    )

    return LaunchDescription([
        # 1. Start YOLOv5 with traffic sign model
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch_path),
            launch_arguments={'model': 'traffic_signs_640s_7_0'}.items()
        ),
        
        # 2. Lane Detection Node (Publishes to /lane_vel)
        Node(
            package='mycar',
            executable='lane_detect_node',
            name='lane_detect',
            output='screen'
        ),
        
        # 3. Mission Manager (Coordinates /lane_vel and YOLO to publish to /cmd_vel)
        Node(
            package='mycar',
            executable='mission_manager',
            name='mission_manager',
            output='screen'
        )
    ])
