import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类

from std_msgs.msg import String  # 导入String消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # 导入QoSProfile和QoSReliabilityPolicy、QoSHistoryPolicy类

class MinimalPublisher(Node):  # 定义一个继承自Node类的MinimalPublisher类

    def __init__(self):
        super().__init__('minimal_publisher')  # 调用父类构造函数初始化节点
        qos_profile = QoSProfile(  # 创建QoSProfile对象
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性策略为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,  # 设置历史策略为KEEP_LAST
            depth=1  # 设置深度为1
        )
        self.publisher_ = self.create_publisher(String, 'topic', qos_profile)  # 创建发布者对象
        timer_period = 0.5  # 设置定时器回调的时间间隔为0.5秒
        self.timer = self.create_timer(timer_period, self.timer_callback)  # 创建定时器，设置回调函数为timer_callback
        self.i = 0

    def timer_callback(self):
        msg = String()  # 创建String消息类型的对象
        msg.data = 'Hello World: %d' % self.i  # 设置消息的数据
        self.publisher_.publish(msg)  # 发布消息
        self.i += 1  # 自增计数器


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS节点

    minimal_publisher = MinimalPublisher()  # 创建MinimalPublisher对象

    rclpy.spin(minimal_publisher)  # 进入主循环

    minimal_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()