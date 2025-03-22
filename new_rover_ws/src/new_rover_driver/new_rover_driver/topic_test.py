#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import time

class TopicTester(Node):
    def __init__(self):
        super().__init__('topic_tester')
        self.get_logger().info('话题测试节点已启动')
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建订阅器
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10)
            
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
            
        self.processed_sub = self.create_subscription(
            Image,
            '/processed_image',
            self.processed_callback,
            10)
            
        self.obstacle_sub = self.create_subscription(
            PointCloud2,
            '/obstacle_points',
            self.obstacle_callback,
            10)
            
        # 创建定时器用于测试发布
        self.create_timer(1.0, self.test_publish)
        
        # 初始化计数器
        self.callbacks_received = {
            'odom': 0,
            'scan': 0,
            'rgb': 0,
            'depth': 0,
            'processed': 0,
            'obstacle': 0
        }
        
    def odom_callback(self, msg):
        self.callbacks_received['odom'] += 1
        if self.callbacks_received['odom'] == 1:
            self.get_logger().info('收到里程计数据')
            self.get_logger().info(f'位置: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
            
    def scan_callback(self, msg):
        self.callbacks_received['scan'] += 1
        if self.callbacks_received['scan'] == 1:
            self.get_logger().info('收到激光扫描数据')
            self.get_logger().info(f'扫描点数: {len(msg.ranges)}')
            
    def rgb_callback(self, msg):
        self.callbacks_received['rgb'] += 1
        if self.callbacks_received['rgb'] == 1:
            self.get_logger().info('收到RGB图像')
            self.get_logger().info(f'图像大小: {msg.width}x{msg.height}')
            
    def depth_callback(self, msg):
        self.callbacks_received['depth'] += 1
        if self.callbacks_received['depth'] == 1:
            self.get_logger().info('收到深度图像')
            self.get_logger().info(f'图像大小: {msg.width}x{msg.height}')
            
    def processed_callback(self, msg):
        self.callbacks_received['processed'] += 1
        if self.callbacks_received['processed'] == 1:
            self.get_logger().info('收到处理后的图像')
            
    def obstacle_callback(self, msg):
        self.callbacks_received['obstacle'] += 1
        if self.callbacks_received['obstacle'] == 1:
            self.get_logger().info('收到障碍物点云数据')
            
    def test_publish(self):
        # 发布测试速度命令
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        
        # 打印状态报告
        self.print_status()
        
    def print_status(self):
        self.get_logger().info('\n话题状态报告:')
        for topic, count in self.callbacks_received.items():
            status = '活跃' if count > 0 else '未接收'
            self.get_logger().info(f'{topic}: {status} (收到 {count} 条消息)')

def main(args=None):
    rclpy.init(args=args)
    tester = TopicTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 