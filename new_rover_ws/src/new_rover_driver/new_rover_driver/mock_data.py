#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
from math import sin, cos, pi
import struct

class MockDataPublisher(Node):
    def __init__(self):
        super().__init__('mock_data_publisher')
        self.get_logger().info('模拟数据发布节点已启动')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)
        
        # 初始化模拟数据参数
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.mock_enabled = False
        
        # 创建定时器
        self.create_timer(0.1, self.publish_odom)      # 10Hz
        self.create_timer(0.1, self.publish_scan)      # 10Hz
        self.create_timer(0.1, self.publish_images)    # 10Hz
        self.create_timer(0.5, self.publish_obstacles) # 2Hz
        
        self.get_logger().info('模拟数据发布器初始化完成')
        
    def enable_mock(self):
        """启用模拟数据发布"""
        self.mock_enabled = True
        self.get_logger().info('模拟数据发布已启用')
        
    def disable_mock(self):
        """禁用模拟数据发布"""
        self.mock_enabled = False
        self.get_logger().info('模拟数据发布已禁用')
        
    def publish_odom(self):
        """发布模拟的里程计数据"""
        if not self.mock_enabled:
            return
            
        # 更新位置
        self.x += 0.01 * cos(self.theta)
        self.y += 0.01 * sin(self.theta)
        self.theta += 0.01
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 设置位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 设置方向（简单的平面旋转）
        odom.pose.pose.orientation.z = sin(self.theta/2.0)
        odom.pose.pose.orientation.w = cos(self.theta/2.0)
        
        self.odom_pub.publish(odom)
        
    def publish_scan(self):
        """发布模拟的激光扫描数据"""
        if not self.mock_enabled:
            return
            
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        # 设置扫描参数
        scan.angle_min = -pi
        scan.angle_max = pi
        scan.angle_increment = pi/180.0  # 1度
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 30.0
        
        # 生成模拟的扫描数据
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = [5.0 + sin(i/30.0) for i in range(num_readings)]
        scan.ranges = ranges
        
        self.scan_pub.publish(scan)
        
    def publish_images(self):
        """发布模拟的图像数据"""
        if not self.mock_enabled:
            return
            
        # 创建模拟的RGB图像
        rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(rgb_image, (320, 240), 100, (0, 255, 0), -1)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(rgb_msg)
        
        # 创建模拟的深度图像
        depth_image = np.ones((480, 640), dtype=np.float32) * 2.0
        cv2.circle(depth_image, (320, 240), 100, 1.0, -1)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(depth_msg)
        
    def publish_obstacles(self):
        """发布模拟的障碍物点云数据"""
        if not self.mock_enabled:
            return
            
        # 创建点云消息
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 创建一些示例点
        points = []
        for i in range(100):
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, 5)
            z = np.random.uniform(0, 2)
            points.append([x, y, z])
            
        # 将点数据转换为numpy数组
        points_array = np.array(points, dtype=np.float32)
        
        # 设置点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 设置点云属性
        msg.height = 1
        msg.width = len(points)
        msg.point_step = 12  # 4 bytes * 3 fields
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        
        # 直接使用numpy数组的字节表示
        msg.data = points_array.tobytes()
        
        self.obstacle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mock_publisher = MockDataPublisher()
    
    try:
        # 等待1秒让节点完全初始化
        time.sleep(1.0)
        # 启用模拟数据发布
        mock_publisher.enable_mock()
        # 运行节点
        rclpy.spin(mock_publisher)
    except KeyboardInterrupt:
        mock_publisher.disable_mock()
        pass
    finally:
        mock_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 