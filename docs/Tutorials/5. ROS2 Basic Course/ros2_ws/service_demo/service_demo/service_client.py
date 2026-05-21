import sys  # 导入sys模块
import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类
from rclpy.logging import get_logger  # 导入get_logger函数
from demo_interfaces.srv import AddInts  # 导入AddInts服务类型

class MinimalClient(Node):  # 定义一个继承自Node类的MinimalClient类

    def __init__(self):
        super().__init__('minimal_client')  # 调用父类构造函数初始化节点
        self.cli = self.create_client(AddInts, 'add_two_ints')  # 创建一个AddInts类型的客户端对象，并连接到'add_two_ints'服务
        while not self.cli.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.get_logger().info('service is connecting...')  # 打印日志信息，表示正在连接服务

    def send_request(self):
        request = AddInts.Request()  # 创建一个AddInts服务请求对象
        request.num1 = int(sys.argv[1])  # 设置请求参数num1为用户输入的第一个整数
        request.num2 = int(sys.argv[2])  # 设置请求参数num2为用户输入的第二个整数
        self.future = self.cli.call_async(request)  # 发送异步服务请求

def main():
    if len(sys.argv) != 3:  # 检查命令行参数是否为两个整数
        get_logger("rclpy").error("please give two integer values")  # 打印错误日志信息，提示用户输入两个整数
        return

    rclpy.init()  # 初始化ROS节点

    minimal_client = MinimalClient()  # 创建MinimalClient对象
    minimal_client.send_request()  # 发送服务请求
    rclpy.spin_until_future_complete(minimal_client, minimal_client.future)  # 进入主循环，直到服务请求完成
    try:
        response = minimal_client.future.result()  # 获取服务响应
        minimal_client.get_logger().info("request result: sum = %d" % response.sum)  # 打印服务响应中的求和结果
    except Exception:
        minimal_client.get_logger().error("request failed")  # 打印错误日志信息，表示服务请求失败

    minimal_client.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()