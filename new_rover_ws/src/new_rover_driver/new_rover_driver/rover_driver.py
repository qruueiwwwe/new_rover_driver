#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from math import cos, sin, pi, atan2
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener, Buffer
import mediapipe as mp
from scipy.spatial import distance

class RoverDriver(Node):
    def __init__(self):
        super().__init__('rover_driver')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 初始化MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
            
        # 初始化TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 初始化位置和方向
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.processed_image_pub = self.create_publisher(Image, '/processed_image', 10)
        self.obstacle_points_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # 创建订阅器
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.voice_cmd_sub = self.create_subscription(String, '/voice_command', self.voice_cmd_callback, 10)
        
        # 创建服务
        self.start_service = self.create_service(Trigger, '/start_driver', self.start_callback)
        self.stop_service = self.create_service(Trigger, '/stop_driver', self.stop_callback)
        self.save_map_service = self.create_service(Trigger, '/save_map', self.save_map_callback)
        self.line_follow_service = self.create_service(SetBool, '/line_follow', self.line_follow_callback)
        
        # 初始化参数
        self.min_distance = 0.3  # 最小检测距离（米）
        self.max_distance = 5.0  # 最大检测距离（米）
        self.obstacle_threshold = 0.5  # 障碍物检测阈值（米）
        self.min_depth = 0.5  # 最小深度值（米）
        self.max_depth = 5.0  # 最大深度值（米）
        
        # 状态标志
        self.is_running = False
        self.is_line_following = False
        self.current_goal = None
        self.path = []
        
        # 初始化地图
        self.map_data = np.zeros((100, 100), dtype=np.int8)
        self.map_resolution = 0.05  # 5cm per pixel
        self.map_origin_x = -2.5
        self.map_origin_y = -2.5
        
        self.get_logger().info('机器人驱动节点已启动')
        
    def odom_callback(self, msg):
        """里程计回调函数"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z
        
        # 更新地图
        self.update_map()
        
        # 发布TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
    def update_map(self):
        """更新地图数据"""
        # 将当前位置转换为地图坐标
        map_x = int((self.position_x - self.map_origin_x) / self.map_resolution)
        map_y = int((self.position_y - self.map_origin_y) / self.map_resolution)
        
        # 确保坐标在有效范围内
        if 0 <= map_x < self.map_data.shape[0] and 0 <= map_y < self.map_data.shape[1]:
            self.map_data[map_y, map_x] = 0  # 0表示自由空间
            
        # 发布地图
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data.shape[0]
        map_msg.info.height = self.map_data.shape[1]
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(map_msg)
        
    def laser_callback(self, msg):
        """激光雷达回调函数"""
        if not self.is_running:
            return
            
        # 处理激光数据
        obstacles = []
        for i, distance in enumerate(msg.ranges):
            if not msg.range_min <= distance <= msg.range_max:
                continue
                
            if distance < self.min_distance or distance > self.max_distance:
                continue
                
            if distance < self.obstacle_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                x = distance * cos(angle)
                y = distance * sin(angle)
                obstacles.append((x, y))
                
                # 更新地图中的障碍物
                map_x = int((x + self.position_x - self.map_origin_x) / self.map_resolution)
                map_y = int((y + self.position_y - self.map_origin_y) / self.map_resolution)
                if 0 <= map_x < self.map_data.shape[0] and 0 <= map_y < self.map_data.shape[1]:
                    self.map_data[map_y, map_x] = 100  # 100表示障碍物
                
        if obstacles:
            self.get_logger().warn(f'检测到 {len(obstacles)} 个障碍物点')
            
    def depth_callback(self, msg):
        """深度相机回调函数"""
        if not self.is_running:
            return
            
        try:
            # 处理深度图像
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 创建深度掩码
            mask = np.logical_and(depth_image > self.min_depth * 1000,
                                depth_image < self.max_depth * 1000)
                                
            # 应用彩色映射
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET)
                
            # 应用掩码
            depth_colormap[~mask] = 0
            
            # 发布处理后的图像
            processed_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {str(e)}')
            
    def image_callback(self, msg):
        """RGB图像回调函数"""
        if not self.is_running:
            return
            
        try:
            # 处理RGB图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # MediaPipe姿态检测
            results = self.pose.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            if results.pose_landmarks:
                # 绘制姿态关键点
                mp.solutions.drawing_utils.draw_landmarks(
                    cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                    
            # 发布处理后的图像
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理RGB图像时出错: {str(e)}')
            
    def goal_callback(self, msg):
        """导航目标回调函数"""
        if not self.is_running:
            return
            
        self.current_goal = msg
        self.path = self.plan_path(msg)
        
        # 发布规划路径
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for point in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        
    def plan_path(self, goal):
        """简单的路径规划"""
        # 这里使用简单的直线路径规划
        # 实际应用中应该使用更复杂的路径规划算法
        start = (self.position_x, self.position_y)
        end = (goal.pose.position.x, goal.pose.position.y)
        return [start, end]
        
    def voice_cmd_callback(self, msg):
        """语音命令回调函数"""
        if not self.is_running:
            return
            
        command = msg.data.lower()
        if "forward" in command:
            self.move_forward(1.0)
        elif "backward" in command:
            self.move_forward(-1.0)
        elif "left" in command:
            self.rotate(pi/2)
        elif "right" in command:
            self.rotate(-pi/2)
        elif "stop" in command:
            self.stop()
            
    def line_follow_callback(self, request, response):
        """巡线功能回调"""
        self.is_line_following = request.data
        response.success = True
        response.message = "巡线功能已" + ("启动" if request.data else "停止")
        return response
        
    def save_map_callback(self, request, response):
        """保存地图回调"""
        try:
            # 保存地图数据
            np.save('map_data.npy', self.map_data)
            response.success = True
            response.message = "地图已保存"
        except Exception as e:
            response.success = False
            response.message = f"保存地图失败: {str(e)}"
        return response
            
    def start_callback(self, request, response):
        """启动驱动服务回调"""
        self.is_running = True
        response.success = True
        response.message = "驱动已启动"
        return response
        
    def stop_callback(self, request, response):
        """停止驱动服务回调"""
        self.is_running = False
        # 停止机器人
        self.stop()
        response.success = True
        response.message = "驱动已停止"
        return response
        
    def move_forward(self, distance, speed=0.2):
        """向前移动指定距离"""
        if not self.is_running:
            return
            
        twist = Twist()
        twist.linear.x = speed
        
        start_x = self.position_x
        start_y = self.position_y
        
        while rclpy.ok() and self.is_running:
            current_distance = ((self.position_x - start_x) ** 2 + 
                              (self.position_y - start_y) ** 2) ** 0.5
            if current_distance >= abs(distance):
                break
                
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop()
        
    def rotate(self, angle, speed=0.5):
        """旋转指定角度（弧度）"""
        if not self.is_running:
            return
            
        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed
        
        start_orientation = self.orientation_z
        target_orientation = start_orientation + angle
        
        while rclpy.ok() and self.is_running:
            if abs(self.orientation_z - target_orientation) < 0.1:
                break
                
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop()
        
    def stop(self):
        """停止机器人"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 