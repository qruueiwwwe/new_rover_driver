#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import math

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver')
        self.get_logger().info('Rover Driver Node Started')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.led_pub = self.create_publisher(Bool, 'led_status', 10)
        self.get_logger().info('Publishers created')
        
        # 创建订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Subscriber created')
            
        # 机器人参数
        self.max_linear_speed = 2.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.wheel_base = 0.3  # 轮距
        self.wheel_radius = 0.05  # 轮子半径
        
        # 状态变量
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.led_status = False
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info('Timer created')
        self.get_logger().info('Initialization complete')
        
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        try:
            # 限制速度在合理范围内
            self.current_linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
            self.current_angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
            
            # 计算左右轮速度
            left_speed = self.current_linear_x - (self.current_angular_z * self.wheel_base / 2)
            right_speed = self.current_linear_x + (self.current_angular_z * self.wheel_base / 2)
            
            # 这里添加实际的电机控制代码
            self.get_logger().info(f'Left wheel speed: {left_speed:.2f}, Right wheel speed: {right_speed:.2f}')
            self.get_logger().debug(f'Received cmd_vel - linear: {msg.linear.x}, angular: {msg.angular.z}')
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')
        
    def publish_status(self):
        """发布机器人状态"""
        try:
            # 发布LED状态
            led_msg = Bool()
            led_msg.data = self.led_status
            self.led_pub.publish(led_msg)
            self.get_logger().debug(f'Published LED status: {self.led_status}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_status: {str(e)}')
        
    def set_led(self, status):
        """设置LED状态"""
        try:
            self.led_status = status
            self.get_logger().info(f'LED status set to: {status}')
        except Exception as e:
            self.get_logger().error(f'Error in set_led: {str(e)}')
        
    def stop(self):
        """停止机器人"""
        try:
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Robot stopped')
        except Exception as e:
            self.get_logger().error(f'Error in stop: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    driver = RoverDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        driver.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        driver.stop()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 