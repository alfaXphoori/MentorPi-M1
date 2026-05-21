from launch import LaunchDescription  # 导入LaunchDescription类，用于描述launch文件
from launch_ros.actions import Node  # 导入Node类，用于启动ROS节点
from ament_index_python.packages import get_package_share_directory  # 导入get_package_share_directory函数，用于获取包的共享目录

def generate_launch_description():
    # 获取URDF文件的路径
    urdf_file_path = get_package_share_directory('urdf_demo') + '/urdf/simple_demo.urdf'

    # 创建参数节点，将URDF文件加载到参数服务器
    load_urdf_param_node = Node(
        package='robot_state_publisher',  # 包名
        executable='robot_state_publisher',  # 可执行文件名
        name='robot_state_publisher',  # 节点名称
        parameters=[{'robot_description': open(urdf_file_path).read()}]  # 将URDF文件加载到参数服务器中
    )

    # 启动RViz节点，并加载URDF模型
    rviz_node = Node(
        package='rviz2',  # 包名
        executable='rviz2',  # 可执行文件名
        name='rviz2',  # 节点名称
        output='screen',  # 输出屏幕信息
        arguments=['-d', get_package_share_directory('urdf_demo') + '/rviz/rviz.rviz']  # 加载RViz配置文件以显示URDF模型
    )

    return LaunchDescription([
        load_urdf_param_node,
        rviz_node
    ])                                                                               

