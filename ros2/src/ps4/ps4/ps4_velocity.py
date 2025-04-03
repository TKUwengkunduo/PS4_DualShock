import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # 參數設定：最大速度與角速度
        self.declare_parameter('max_linear_speed', 0.5)   # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s

        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # 初始化 Pygame 與搖桿
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No joystick found. Please connect a controller.')
            rclpy.shutdown()
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Joystick connected: {self.joystick.get_name()}')

        # 使用定時器讀取搖桿輸入
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.joystick_callback)

    def joystick_callback(self):
        pygame.event.pump()  # 更新搖桿狀態

        # PS4 左搖桿控制速度：X 軸是左右，Y 軸是前後（反轉 Y）
        axis_left_x = self.joystick.get_axis(0)  # 左搖桿左右
        axis_left_y = self.joystick.get_axis(1)  # 左搖桿前後（反轉）

        # Deadzone 處理
        if abs(axis_left_x) < 0.2:
            axis_left_x = 0.0
        if abs(axis_left_y) < 0.2:
            axis_left_y = 0.0

        twist = Twist()
        twist.linear.x = axis_left_y * self.max_linear_speed
        twist.angular.z = axis_left_x * self.max_angular_speed

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickTeleop()
    try:
        rclpy.spin(joystick_node)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
