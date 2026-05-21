import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类

from std_msgs.msg import String  # 导入String消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # 导入QoSProfile和QoSReliabilityPolicy、QoSHistoryPolicy类

class MinimalSubscriber(Node):  # 定义一个继承自Node类的MinimalSubscriber类

    def __init__(self):
        super().__init__('minimal_subscriber')  # 调用父类构造函数初始化节点
        qos_profile = QoSProfile(  # 创建QoSProfile对象
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性策略为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,  # 设置历史策略为KEEP_LAST
            depth=1  # 设置深度为1
        )
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, qos_profile)  # 创建订阅者对象，并设置回调函数为listener_callback

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)  # 打印接收到的消息内容


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS节点

    minimal_subscriber = MinimalSubscriber()  # 创建MinimalSubscriber对象

    rclpy.spin(minimal_subscriber)  # 进入主循环

    minimal_subscriber.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()