import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# import keyboard  # 用于监听键盘输入
import readchar
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/gimbal_ref', 10)
        self.twist = Twist()

        self.get_logger().info("使用 W/A/S/D 控制机器人移动，Q 退出")

        # 速度参数
        self.linear_speed = 0.5  # 线速度
        self.angular_speed = 1.0  # 角速度

    def update_twist(self, key):
        """ 根据按键更新 Twist 消息 """
        if key == 'w':  # 前进
            # self.twist.linear.x = self.linear_speed
            # self.twist.angular.z = 0.0
            self.twist.angular.y += self.angular_speed

        elif key == 's':  # 后退
            self.twist.angular.y -= self.angular_speed
            # self.twist.linear.x = -self.linear_speed
            # self.twist.angular.z = 0.0
        elif key == 'a':  # 左转
            # self.twist.linear.x = 0.0
            self.twist.angular.z += self.angular_speed
        elif key == 'd':  # 右转
            # self.twist.linear.x = 0.0
            self.twist.angular.z -= self.angular_speed
        elif key == 'q':  # 退出
            self.twist.linear.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)
            self.get_logger().info("退出控制")
            return False  # 退出循环
        else:  # 停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.publisher_.publish(self.twist)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            key = readchar.readkey()
            if key == 'q':  # 按 Q 退出
                break
            elif key in ('w', 's', 'a', 'd'):
                node.update_twist(key)
            #     node.update_twist('w')
            # elif keyboard.is_pressed('s'):
            #     node.update_twist('s')
            # elif keyboard.is_pressed('a'):
            #     node.update_twist('a')
            # elif keyboard.is_pressed('d'):
            #     node.update_twist('d')
            # else:
            #     node.update_twist('')  # 没有按键则停止
            
            rclpy.spin_once(node, timeout_sec=0.1)  # 避免过高 CPU 占用
    except KeyboardInterrupt:
        node.get_logger().info("手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
