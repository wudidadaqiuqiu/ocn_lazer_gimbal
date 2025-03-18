import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import readchar
import threading

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/gimbal_ref', 10)
        self.twist = Twist()

        self.get_logger().info("使用 W/A/S/D 控制机器人移动，Q 退出")

        # 速度参数
        self.linear_speed = 0.5  # 线速度
        self.angular_speed = 1.0  # 角速度
        self.running = True  # 控制线程的运行状态
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1秒发布一次

        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def timer_callback(self):
        """ 定时发布 Twist 消息 """
        self.publisher_.publish(self.twist)

    def update_twist(self, key):
        """ 根据按键更新 Twist 消息 """
        if key == 'w':  # 前进
            self.twist.angular.y += self.angular_speed
        elif key == 's':  # 后退
            self.twist.angular.y -= self.angular_speed
        elif key == 'a':  # 左转
            self.twist.angular.z += self.angular_speed
        elif key == 'd':  # 右转
            self.twist.angular.z -= self.angular_speed
        elif key == 'q':  # 退出
            self.get_logger().info("退出控制")
            self.running = False  # 让键盘监听线程退出
        else:  # 其他按键停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

    def keyboard_listener(self):
        """ 独立线程监听键盘输入 """
        while self.running and rclpy.ok():
            key = readchar.readkey()
            self.update_twist(key)
            if key == 'q':
                break  # 退出监听循环

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)  # 让 ROS 2 运行
    except KeyboardInterrupt:
        node.get_logger().info("手动终止")
    finally:
        node.running = False  # 停止键盘监听线程
        node.keyboard_thread.join()  # 等待线程退出
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
