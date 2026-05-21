import rclpy  # 导入rclpy模块
from rclpy.node import Node  # 导入Node类
from rclpy.action import ActionClient  # 导入ActionClient类
from demo_interfaces.action import FileDownload  # 导入FileDownload action接口

class FileDownloadActionClient(Node):  # 定义一个继承自Node类的FileDownloadActionClient类
    def __init__(self):
        super().__init__('file_download_action_client')  # 调用父类构造函数初始化节点
        self._action_client = ActionClient(self, FileDownload, 'file_download')  # 创建ActionClient对象

    def send_goal(self, file_size):
        goal_msg = FileDownload.Goal()  # 创建Goal消息类型的对象
        goal_msg.file_size = file_size  # 设置文件大小

        self.get_logger().info(f'Sending file download goal for {file_size} bytes')  # 打印发送目标的日志

        self._action_client.wait_for_server()  # 等待服务器可用
        self.future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)  # 发送目标异步请求
        
        self.future.add_done_callback(self.goal_response_callback)  # 添加目标响应回调函数

    def goal_response_callback(self,future):
        goal_handle = future.result()  # 获取目标句柄
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")  # 打印目标被拒绝的日志
            return
        self.get_logger().info("Goal accepted")  # 打印目标被接受的日志
        self._get_result_future = goal_handle.get_result_async()  # 获取目标结果异步请求
        self._get_result_future.add_done_callback(self.get_result_callback)  # 添加获取结果的回调函数
        
    def get_result_callback(self,future):
        self.get_logger().info("File download completed successfully.")  # 打印文件下载成功完成的日志

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.completion_percentage:.2f}% downloaded')  # 打印接收到的反馈消息

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS节点
    client = FileDownloadActionClient()  # 创建FileDownloadActionClient对象

    result = client.send_goal(100)  # 发送目标

    rclpy.spin(client)  # 进入主循环
    client.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS

if __name__ == '__main__':
    main()