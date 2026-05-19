from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='mycar', executable='fsd_lane_keep', name='fsd_lane', output='screen'),
        Node(package='mycar', executable='fsd_lidar_safety', name='fsd_safety', output='screen'),
        Node(package='mycar', executable='fsd_mission_manager', name='fsd_manager', output='screen'),
        Node(package='mycar', executable='yolo_logic_node', name='yolo_logic', output='screen'),
    ])
