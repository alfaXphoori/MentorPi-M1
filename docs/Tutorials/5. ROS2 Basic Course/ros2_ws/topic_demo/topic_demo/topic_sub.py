import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类

from std_msgs.msg import String  # 导入String消息类型


class MinimalSubscriber(Node):  # 定义一个继承自Node类的MinimalSubscriber类

    def __init__(self):
        super().__init__('minimal_subscriber')  # 调用父类构造函数初始化节点
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)  # 创建一个订阅者对象，订阅名为'topic'的话题的String类型消息，队列大小为10

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