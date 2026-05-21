import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类

from std_msgs.msg import String  # 导入String消息类型


class MinimalPublisher(Node):  # 定义一个继承自Node类的MinimalPublisher类

    def __init__(self):
        super().__init__('minimal_publisher')  # 调用父类构造函数初始化节点
        self.publisher_ = self.create_publisher(String, 'topic', 10)  # 创建一个发布者对象，发布String类型的消息到名为'topic'的话题，队列大小为10
        timer_period = 0.5  # 定时器周期为0.5秒
        self.timer = self.create_timer(timer_period, self.timer_callback)  # 创建定时器，设置周期为timer_period，回调函数为timer_callback
        self.i = 0  # 初始化计数器

    def timer_callback(self):
        msg = String()  # 创建一个String类型的消息对象
        msg.data = 'Hello World: %d' % self.i  # 设置消息内容为'Hello World: i'，其中i为计数器的值
        self.publisher_.publish(msg)  # 发布消息到话题
        self.i += 1  # 计数器自增1


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS节点

    minimal_publisher = MinimalPublisher()  # 创建MinimalPublisher对象

    rclpy.spin(minimal_publisher)  # 进入主循环

    minimal_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS


if __name__ == '__main__':
    main()