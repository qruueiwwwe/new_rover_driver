#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.clock import Clock
# from geometry_msgs.msg import Twist, PoseStamped, Path
from sensor_msgs.msg import LaserScan, Image, Imu, MagneticField, JointState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Int32, Float32
from std_srvs.srv import SetBool
import cv2
import numpy as np
import mediapipe as mp
import math
import threading
import time
import random

class RoverDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 初始化参数
        self.declare_parameter('min_distance', 0.5)  # 最小安全距离
        self.declare_parameter('max_distance', 3.0)  # 最大检测距离
        self.declare_parameter('obstacle_threshold', 0.3)  # 障碍物阈值
        self.declare_parameter('min_depth', 0.1)  # 最小深度值
        self.declare_parameter('max_depth', 3.0)  # 最大深度值
        self.declare_parameter('map_resolution', 0.05)  # 地图分辨率
        self.declare_parameter('map_width', 1000)  # 地图宽度
        self.declare_parameter('map_height', 1000)  # 地图高度
        
        # 获取参数值
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        
        # 初始化状态变量
        self.is_running = False
        # self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.orientation.w = 1.0
        self.current_velocity = Twist()
        self.latest_scan = None
        self.latest_depth = None
        self.latest_rgb = None
        self.latest_imu = None
        self.latest_mag = None
        self.battery_voltage = 0.0
        self.car_version = 1.0
        self.is_following_line = False
        self.current_steer_angle = 0.0
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.battery_pub = self.create_publisher(Float32, 'voltage', 100)
        self.version_pub = self.create_publisher(Float32, 'edition', 100)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 100)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 100)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 100)
        self.buzzer_pub = self.create_publisher(Bool, 'Buzzer', 1)
        self.rgb_light_pub = self.create_publisher(Int32, 'RGBLight', 10)
        
        # 创建订阅器
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'camera/depth/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, 'camera/rgb/image_raw', self.rgb_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)
        # self.goal_sub = self.create_subscription(PoseStamped, 'move_base_simple/goal', self.goal_callback, 10)
        self.voice_cmd_sub = self.create_subscription(Int32, 'voice_command', self.voice_command_callback, 10)
        
        # 创建服务
        self.start_srv = self.create_service(SetBool, 'start_driver', self.start_driver_callback)
        self.stop_srv = self.create_service(SetBool, 'stop_driver', self.stop_driver_callback)
        self.save_map_srv = self.create_service(SetBool, 'save_map', self.save_map_callback)
        self.line_follow_srv = self.create_service(SetBool, 'line_follow', self.line_follow_callback)
        
        # 初始化MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始化地图
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.data = [0] * (self.map_width * self.map_height)
        
        # 创建线程锁
        self.lock = threading.Lock()
        
        self.get_logger().info('RoverDriver节点已初始化完成')

    def timer_callback(self):
        """定时器回调函数，用于发布状态信息"""
        if not self.is_running:
            return
            
        # 发布电池电压
        battery_msg = Float32()
        battery_msg.data = self.battery_voltage
        self.battery_pub.publish(battery_msg)
        
        # 发布版本信息
        version_msg = Float32()
        version_msg.data = self.car_version
        self.version_pub.publish(version_msg)
        
        # 发布IMU数据
        if self.latest_imu:
            self.imu_pub.publish(self.latest_imu)
            
        # 发布磁力计数据
        if self.latest_mag:
            self.mag_pub.publish(self.latest_mag)
            
        # 发布关节状态
        joint_state = JointState()
        joint_state.header.stamp = Clock().now().to_msg()
        joint_state.header.frame_id = "joint_states"
        joint_state.name = ["back_right_joint", "back_left_joint", 
                          "front_left_steer_joint", "front_left_wheel_joint",
                          "front_right_steer_joint", "front_right_wheel_joint"]
        
        # 计算转向角度（弧度）
        steer_radis = self.current_steer_angle * math.pi / 180.0
        joint_state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
        
        # 如果小车在运动，添加随机轮子位置
        if not (self.current_velocity.linear.x == 0 and 
                self.current_velocity.angular.z == 0):
            i = random.uniform(-math.pi, math.pi)
            joint_state.position = [i, i, steer_radis, i, steer_radis, i]
            
        self.joint_state_pub.publish(joint_state)
        
        # 更新地图
        self.update_map()
        
        # 发布地图
        self.map_pub.publish(self.map)

    def update_map(self):
        """更新地图信息"""
        with self.lock:
            # 更新机器人位置
            robot_x = int(self.current_pose.pose.position.x / self.map_resolution)
            robot_y = int(self.current_pose.pose.position.y / self.map_resolution)
            
            # 更新障碍物信息
            if self.latest_scan:
                for i, range_value in enumerate(self.latest_scan.ranges):
                    if range_value < self.max_distance and range_value > self.min_distance:
                        angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
                        x = int(robot_x + range_value * math.cos(angle) / self.map_resolution)
                        y = int(robot_y + range_value * math.sin(angle) / self.map_resolution)
                        if 0 <= x < self.map_width and 0 <= y < self.map_height:
                            self.map.data[y * self.map_width + x] = 100  # 标记为障碍物

    def scan_callback(self, msg):
        """激光雷达数据回调"""
        with self.lock:
            self.latest_scan = msg
            # 检查前方障碍物
            if self.is_running:
                self.check_obstacles()

    def depth_callback(self, msg):
        """深度相机数据回调"""
        with self.lock:
            self.latest_depth = msg
            # 处理深度图像
            if self.is_running:
                self.process_depth_image()

    def rgb_callback(self, msg):
        """RGB相机数据回调"""
        with self.lock:
            self.latest_rgb = msg
            # 处理RGB图像
            if self.is_running:
                self.process_rgb_image()

    def imu_callback(self, msg):
        """IMU数据回调"""
        with self.lock:
            self.latest_imu = msg
            # 更新姿态信息
            self.current_pose.pose.orientation = msg.orientation

    def mag_callback(self, msg):
        """磁力计数据回调"""
        with self.lock:
            self.latest_mag = msg

    def goal_callback(self, msg):
        """导航目标回调"""
        if not self.is_running:
            return
            
        # 简单的路径规划
        path = Path()
        path.header.frame_id = "map"
        path.poses.append(self.current_pose)
        path.poses.append(msg)
        
        # 发布规划路径
        self.path_pub.publish(path)
        
        # 执行运动
        self.move_to_goal(msg)

    def voice_command_callback(self, msg):
        """语音命令回调"""
        if not self.is_running:
            return
            
        if msg.data == 2 or msg.data == 0:  # 停止
            self.stop()
        elif msg.data == 4:  # 前进
            self.move_forward(0.5)
            time.sleep(5)
            self.stop()
        elif msg.data == 5:  # 后退
            self.move_backward(0.5)
            time.sleep(5)
            self.stop()
        elif msg.data == 6:  # 左转
            self.rotate(0.2)
            time.sleep(5)
            self.stop()
        elif msg.data == 7:  # 右转
            self.rotate(-0.2)
            time.sleep(5)
            self.stop()
        elif msg.data == 11:  # 红灯
            self.set_rgb_light(255, 0, 0)
        elif msg.data == 12:  # 绿灯
            self.set_rgb_light(0, 255, 0)
        elif msg.data == 13:  # 蓝灯
            self.set_rgb_light(0, 0, 255)
        elif msg.data == 14:  # 黄灯
            self.set_rgb_light(255, 255, 0)
        elif msg.data == 15:  # 流水灯
            self.set_rgb_effect(1)
        elif msg.data == 16:  # 渐变灯
            self.set_rgb_effect(4)
        elif msg.data == 17:  # 呼吸灯
            self.set_rgb_effect(3)
        elif msg.data == 18:  # 显示电量
            self.set_rgb_effect(6)

    def start_driver_callback(self, request, response):
        """启动驱动服务回调"""
        if request.data:
            self.is_running = True
            response.success = True
            response.message = "驱动已启动"
        else:
            self.is_running = False
            response.success = True
            response.message = "驱动已停止"
        return response

    def stop_driver_callback(self, request, response):
        """停止驱动服务回调"""
        if request.data:
            self.is_running = False
            self.stop()
            response.success = True
            response.message = "驱动已停止"
        return response

    def save_map_callback(self, request, response):
        """保存地图服务回调"""
        if request.data:
            # TODO: 实现地图保存功能
            response.success = True
            response.message = "地图已保存"
        return response

    def line_follow_callback(self, request, response):
        """循线控制服务回调"""
        if request.data:
            self.is_following_line = True
            response.success = True
            response.message = "循线模式已启动"
        else:
            self.is_following_line = False
            response.success = True
            response.message = "循线模式已停止"
        return response

    def check_obstacles(self):
        """检查障碍物"""
        if not self.latest_scan:
            return
            
        # 检查前方障碍物
        front_ranges = self.latest_scan.ranges[len(self.latest_scan.ranges)//3:2*len(self.latest_scan.ranges)//3]
        min_distance = min(front_ranges)
        
        if min_distance < self.min_distance:
            self.stop()
            self.get_logger().warn(f'检测到前方障碍物，距离: {min_distance:.2f}m')

    def process_depth_image(self):
        """处理深度图像"""
        if not self.latest_depth:
            return
            
        try:
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            
            # 将深度值转换为米单位（如果需要）
            depth_meters = depth_image.astype(float) / 1000.0
            
            # 创建深度掩码
            mask = np.logical_and(depth_meters > self.min_depth,
                                depth_meters < self.max_depth)
            
            # 对深度图进行归一化以便可视化
            normalized_depth = cv2.normalize(depth_meters, None, 0, 255, cv2.NORM_MINMAX)
            normalized_depth = normalized_depth.astype(np.uint8)
            
            # 应用彩色映射
            depth_colormap = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
            
            # 应用掩码
            depth_colormap[~mask] = 0
            
            # 检测障碍物
            obstacles = depth_meters < self.obstacle_threshold
            if np.any(obstacles):
                obstacle_positions = np.where(obstacles)
                self.get_logger().warn(f'检测到 {len(obstacle_positions[0])} 个障碍物点')
                
                # 如果中心区域有障碍物，发出警告
                center_region = obstacles[
                    depth_meters.shape[0]//3:2*depth_meters.shape[0]//3,
                    depth_meters.shape[1]//3:2*depth_meters.shape[1]//3
                ]
                if np.any(center_region):
                    self.get_logger().warn('前方存在障碍物！')
                    # 如果机器人正在移动，则停止
                    if abs(self.current_velocity.linear.x) > 0 or abs(self.current_velocity.angular.z) > 0:
                        self.stop()
            
            # 添加文本信息
            cv2.putText(depth_colormap, 
                       f'Min depth: {np.min(depth_meters):.2f}m', 
                       (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       1, 
                       (255, 255, 255), 
                       2)
            cv2.putText(depth_colormap, 
                       f'Max depth: {np.max(depth_meters):.2f}m', 
                       (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       1, 
                       (255, 255, 255), 
                       2)
            
            # 发布处理后的图像
            processed_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
            processed_msg.header = self.latest_depth.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {str(e)}')

    def process_rgb_image(self):
        """处理RGB图像"""
        if not self.latest_rgb:
            return
            
        # 转换图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
            
            # 使用MediaPipe进行姿态检测
            results = self.pose.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            if results.pose_landmarks:
                # 绘制关键点
                mp_drawing = mp.solutions.drawing_utils
                mp_drawing.draw_landmarks(
                    cv_image,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS
                )
                
                # 发布处理后的图像
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')

    def move_to_goal(self, goal):
        """移动到目标位置"""
        # 计算当前位置到目标的距离和角度
        dx = goal.pose.position.x - self.current_pose.pose.position.x
        dy = goal.pose.position.y - self.current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        # 简单的运动控制
        if distance > 0.1:
            # 调整朝向
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
            angle_diff = angle - current_yaw
            if abs(angle_diff) > 0.1:
                self.rotate(angle_diff)
            else:
                # 前进
                self.move_forward(min(distance, 0.5))
        else:
            self.stop()

    def get_yaw_from_quaternion(self, quaternion):
        """从四元数获取偏航角"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # 计算偏航角
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        return yaw

    def move_forward(self, speed=0.5):
        """前进"""
        cmd = Twist()
        cmd.linear.x = speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def move_backward(self, speed=0.5):
        """后退"""
        cmd = Twist()
        cmd.linear.x = -speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def rotate(self, angular_speed):
        """旋转"""
        cmd = Twist()
        cmd.angular.z = angular_speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """停止"""
        cmd = Twist()
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def set_rgb_light(self, r, g, b):
        """设置RGB灯颜色"""
        msg = Int32()
        msg.data = (r << 16) | (g << 8) | b
        self.rgb_light_pub.publish(msg)

    def set_rgb_effect(self, effect):
        """设置RGB灯效果"""
        msg = Int32()
        msg.data = effect
        self.rgb_light_pub.publish(msg)

def main():
    rclpy.init()
    driver = RoverDriver('rover_driver')
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 