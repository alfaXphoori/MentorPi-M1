import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类
from demo_interfaces.srv import AddInts  # 导入AddInts服务类型

class MinimalService(Node):  # 定义一个继承自Node类的MinimalService类

    def __init__(self):
        super().__init__('minimal_service')  # 调用父类构造函数初始化节点
        self.srv = self.create_service(AddInts, 'add_two_ints', self.add_two_ints_callback)  # 创建一个AddInts类型的服务对象，并注册回调函数为add_two_ints_callback

    def add_two_ints_callback(self, request, response):
        response.sum = request.num1 + request.num2  # 计算两个整数的和，并将结果赋给响应对象的sum字段
        self.get_logger().info('Incoming request\nnum1: %d num2: %d' % (request.num1, request.num2))  # 打印接收到的请求内容

        return response  # 返回响应对象

def main():
    rclpy.init()  # 初始化ROS节点

    minimal_service = MinimalService()  # 创建MinimalService对象

    rclpy.spin(minimal_service)  # 进入主循环

    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()