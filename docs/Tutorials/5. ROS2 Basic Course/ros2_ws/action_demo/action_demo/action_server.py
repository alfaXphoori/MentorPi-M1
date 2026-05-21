import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类
from rclpy.action import ActionServer  # 导入ActionServer类
from demo_interfaces.action import FileDownload  # 导入FileDownload action接口
import random  # 导入random模块

class FileDownloadActionServer(Node):  # 定义一个继承自Node类的FileDownloadActionServer类
    def __init__(self):
        super().__init__('file_download_action_server')  # 调用父类构造函数初始化节点
        self._action_server = ActionServer(
            self,
            FileDownload,  # 使用FileDownload action接口
            'file_download',  # 定义action的名称为file_download
            self.execute_callback)  # 设置回调函数为execute_callback

    def execute_callback(self, goal_handle):
        # 服务器端执行的回调函数

        self.get_logger().info(f'Start file download for {goal_handle.request.file_size} bytes...')  # 打印开始下载的日志
        feedback_msg = FileDownload.Feedback()  # 创建Feedback消息类型的对象

        current_size = 0  # 初始化当前下载的文件大小为0
        while current_size < goal_handle.request.file_size:
            increment_size = random.randint(1, 10)  # 模拟随机增加下载大小
            current_size += increment_size  # 更新当前下载的文件大小
            if current_size > goal_handle.request.file_size:
                current_size = goal_handle.request.file_size
            completion_percentage = (current_size / goal_handle.request.file_size) * 100  # 计算下载进度百分比

            feedback_msg.completion_percentage = completion_percentage  # 更新Feedback消息的下载进度
            self.get_logger().info(f'Publishing feedback: {completion_percentage:.2f}% downloaded')  # 打印发布的反馈消息
            goal_handle.publish_feedback(feedback_msg)  # 发布反馈消息
            rclpy.spin_once(self, timeout_sec=1.0)  # 模拟时间的流逝

        goal_handle.succeed()  # 指示目标已完成
        result = FileDownload.Result()  # 创建Result消息类型的对象
        result.current_size = current_size  # 设置Result消息的当前文件大小
        self.get_logger().info('File download completed！')  # 打印文件下载完成的日志
        return result  # 返回结果

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS节点
    server = FileDownloadActionServer()  # 创建FileDownloadActionServer对象
    rclpy.spin(server)  # 进入主循环
    server.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()