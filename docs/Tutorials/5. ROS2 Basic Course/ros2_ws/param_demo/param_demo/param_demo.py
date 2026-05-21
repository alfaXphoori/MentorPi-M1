import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类
from rclpy.parameter import Parameter  # 导入Parameter类

class MinimalParam(Node):  # 定义一个继承自Node类的MinimalParam类
    def __init__(self):
        super().__init__('minimal_param_node')  # 调用父类构造函数初始化节点

        self.declare_parameter('my_parameter', 'hiwonder')  # 声明一个名为'my_parameter'的参数，并设置默认值为'hiwonder'

        self.timer = self.create_timer(1, self.timer_callback)  # 创建定时器，设置回调函数为timer_callback，时间间隔为1秒

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value  # 获取参数'my_parameter'的值，并转换为字符串

        self.get_logger().info('Hello %s!' % my_param)  # 打印带有参数值的日志消息

        my_new_param = Parameter(  # 创建一个新的参数对象
            'my_parameter',  # 参数名称为'my_parameter'
            rclpy.Parameter.Type.STRING,  # 参数类型为字符串
            'hiwonder'  # 参数值为'hiwonder'
        )
        all_new_parameters = [my_new_param]  # 将新的参数对象放入列表中
        self.set_parameters(all_new_parameters)  # 设置节点的参数值为新的参数值

def main():
    rclpy.init()  # 初始化ROS节点
    node = MinimalParam()  # 创建MinimalParam对象
    rclpy.spin(node)  # 进入主循环

if __name__ == '__main__':
    main()