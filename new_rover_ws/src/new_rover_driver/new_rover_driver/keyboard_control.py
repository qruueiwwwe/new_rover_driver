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
        self.get_logger().info('Created cmd_vel publisher')
        
        # 速度参数
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 创建键盘控制线程
        self.running = True
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
        
        while self.running and rclpy.ok():
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = self.get_key()
                cmd_vel = Twist()
                
                if key == 'w':
                    cmd_vel.linear.x = self.linear_speed
                    self.get_logger().info(f'Forward: {self.linear_speed} m/s')
                elif key == 's':
                    cmd_vel.linear.x = -self.linear_speed
                    self.get_logger().info(f'Backward: {-self.linear_speed} m/s')
                elif key == 'a':
                    cmd_vel.angular.z = self.angular_speed
                    self.get_logger().info(f'Left turn: {self.angular_speed} rad/s')
                elif key == 'd':
                    cmd_vel.angular.z = -self.angular_speed
                    self.get_logger().info(f'Right turn: {-self.angular_speed} rad/s')
                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    self.running = False
                    break
                
                try:
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.get_logger().debug(f'Published: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')
                except Exception as e:
                    self.get_logger().error(f'Failed to publish cmd_vel: {str(e)}')
            
            time.sleep(0.1)
            
        # 停止机器人
        cmd_vel = Twist()
        try:
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Robot stopped')
        except Exception as e:
            self.get_logger().error(f'Failed to stop robot: {str(e)}')
        
def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        keyboard_control.get_logger().info('Keyboard interrupt received')
    finally:
        keyboard_control.running = False
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 