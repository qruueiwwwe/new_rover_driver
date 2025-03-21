#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading
import time

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.get_logger().info('Keyboard Control Node Started')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 速度参数
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 创建键盘控制线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
    def get_key(self):
        """获取键盘输入"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def keyboard_loop(self):
        """键盘控制循环"""
        self.get_logger().info('Keyboard Control Started')
        self.get_logger().info('Use WASD keys to control the robot')
        self.get_logger().info('Press Q to quit')
        
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = self.get_key()
                cmd_vel = Twist()
                
                if key == 'w':
                    cmd_vel.linear.x = self.linear_speed
                elif key == 's':
                    cmd_vel.linear.x = -self.linear_speed
                elif key == 'a':
                    cmd_vel.angular.z = self.angular_speed
                elif key == 'd':
                    cmd_vel.angular.z = -self.angular_speed
                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    break
                else:
                    continue
                    
                self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)
            
        # 停止机器人
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        
def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 