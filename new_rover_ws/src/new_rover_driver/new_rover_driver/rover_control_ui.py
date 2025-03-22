#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QPushButton, 
                           QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, 
                           QSlider, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from PyQt5.QtGui import QImage, QPixmap

class RoverControlUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.initUI()
        self.initROS()
        
    def initROS(self):
        # 创建发布器
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.rgb_light_pub = self.node.create_publisher(Int32, 'RGBLight', 10)
        self.buzzer_pub = self.node.create_publisher(Bool, 'Buzzer', 1)
        self.voice_cmd_pub = self.node.create_publisher(Int32, 'voice_command', 10)
        
        # 创建订阅器
        self.image_sub = self.node.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            10)
        self.voltage_sub = self.node.create_subscription(
            Float32,
            'voltage',
            self.voltage_callback,
            10)
            
        # 创建定时器，用于更新UI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # 每100ms更新一次
        
    def initUI(self):
        self.setWindowTitle('机器人控制面板')
        self.setGeometry(100, 100, 800, 600)
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout()
        
        # 左侧控制面板
        left_panel = QVBoxLayout()
        
        # 运动控制组
        motion_group = QGroupBox('运动控制')
        motion_layout = QGridLayout()
        
        # 运动控制按钮
        self.btn_forward = QPushButton('前进')
        self.btn_backward = QPushButton('后退')
        self.btn_left = QPushButton('左转')
        self.btn_right = QPushButton('右转')
        self.btn_stop = QPushButton('停止')
        
        # 设置按钮样式
        motion_btns = [self.btn_forward, self.btn_backward, 
                      self.btn_left, self.btn_right, self.btn_stop]
        for btn in motion_btns:
            btn.setMinimumSize(80, 80)
            btn.setFont(QFont('Arial', 12))
        
        # 添加运动控制按钮到网格布局
        motion_layout.addWidget(self.btn_forward, 0, 1)
        motion_layout.addWidget(self.btn_left, 1, 0)
        motion_layout.addWidget(self.btn_stop, 1, 1)
        motion_layout.addWidget(self.btn_right, 1, 2)
        motion_layout.addWidget(self.btn_backward, 2, 1)
        
        motion_group.setLayout(motion_layout)
        
        # RGB灯光控制组
        light_group = QGroupBox('灯光控制')
        light_layout = QGridLayout()
        
        # RGB灯光控制按钮
        self.btn_red = QPushButton('红灯')
        self.btn_green = QPushButton('绿灯')
        self.btn_blue = QPushButton('蓝灯')
        self.btn_yellow = QPushButton('黄灯')
        self.btn_flow = QPushButton('流水灯')
        self.btn_breath = QPushButton('呼吸灯')
        
        # 添加灯光控制按钮到网格布局
        light_layout.addWidget(self.btn_red, 0, 0)
        light_layout.addWidget(self.btn_green, 0, 1)
        light_layout.addWidget(self.btn_blue, 0, 2)
        light_layout.addWidget(self.btn_yellow, 1, 0)
        light_layout.addWidget(self.btn_flow, 1, 1)
        light_layout.addWidget(self.btn_breath, 1, 2)
        
        light_group.setLayout(light_layout)
        
        # 功能控制组
        function_group = QGroupBox('功能控制')
        function_layout = QVBoxLayout()
        
        self.btn_start = QPushButton('启动驱动')
        self.btn_line_follow = QPushButton('启动循线')
        self.btn_save_map = QPushButton('保存地图')
        
        function_layout.addWidget(self.btn_start)
        function_layout.addWidget(self.btn_line_follow)
        function_layout.addWidget(self.btn_save_map)
        
        function_group.setLayout(function_layout)
        
        # 状态显示组
        status_group = QGroupBox('状态信息')
        status_layout = QVBoxLayout()
        
        self.voltage_label = QLabel('电池电压: 0.0V')
        self.status_label = QLabel('状态: 未启动')
        
        status_layout.addWidget(self.voltage_label)
        status_layout.addWidget(self.status_label)
        
        status_group.setLayout(status_layout)
        
        # 添加所有组到左侧面板
        left_panel.addWidget(motion_group)
        left_panel.addWidget(light_group)
        left_panel.addWidget(function_group)
        left_panel.addWidget(status_group)
        
        # 右侧图像显示面板
        right_panel = QVBoxLayout()
        self.image_label = QLabel()
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.image_label)
        
        # 将左右面板添加到主布局
        main_layout.addLayout(left_panel)
        main_layout.addLayout(right_panel)
        
        # 设置主布局
        central_widget.setLayout(main_layout)
        
        # 连接信号和槽
        self.connectSignals()
        
    def connectSignals(self):
        # 运动控制
        self.btn_forward.pressed.connect(lambda: self.move_robot(0.5, 0))
        self.btn_forward.released.connect(self.stop_robot)
        self.btn_backward.pressed.connect(lambda: self.move_robot(-0.5, 0))
        self.btn_backward.released.connect(self.stop_robot)
        self.btn_left.pressed.connect(lambda: self.move_robot(0, 0.5))
        self.btn_left.released.connect(self.stop_robot)
        self.btn_right.pressed.connect(lambda: self.move_robot(0, -0.5))
        self.btn_right.released.connect(self.stop_robot)
        self.btn_stop.clicked.connect(self.stop_robot)
        
        # 灯光控制
        self.btn_red.clicked.connect(lambda: self.set_light(11))
        self.btn_green.clicked.connect(lambda: self.set_light(12))
        self.btn_blue.clicked.connect(lambda: self.set_light(13))
        self.btn_yellow.clicked.connect(lambda: self.set_light(14))
        self.btn_flow.clicked.connect(lambda: self.set_light(15))
        self.btn_breath.clicked.connect(lambda: self.set_light(17))
        
        # 功能控制
        self.btn_start.clicked.connect(self.toggle_driver)
        self.btn_line_follow.clicked.connect(self.toggle_line_follow)
        self.btn_save_map.clicked.connect(self.save_map)
        
    def move_robot(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def set_light(self, command):
        msg = Int32()
        msg.data = command
        self.rgb_light_pub.publish(msg)
        
    def toggle_driver(self):
        if self.btn_start.text() == '启动驱动':
            self.node.create_client(SetBool, 'start_driver').call_async(SetBool.Request(data=True))
            self.btn_start.setText('停止驱动')
            self.status_label.setText('状态: 已启动')
        else:
            self.node.create_client(SetBool, 'stop_driver').call_async(SetBool.Request(data=True))
            self.btn_start.setText('启动驱动')
            self.status_label.setText('状态: 未启动')
            
    def toggle_line_follow(self):
        if self.btn_line_follow.text() == '启动循线':
            self.node.create_client(SetBool, 'line_follow').call_async(SetBool.Request(data=True))
            self.btn_line_follow.setText('停止循线')
        else:
            self.node.create_client(SetBool, 'line_follow').call_async(SetBool.Request(data=False))
            self.btn_line_follow.setText('启动循线')
            
    def save_map(self):
        self.node.create_client(SetBool, 'save_map').call_async(SetBool.Request(data=True))
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            self.node.get_logger().error(f'图像处理错误: {str(e)}')
            
    def voltage_callback(self, msg):
        self.voltage_label.setText(f'电池电压: {msg.data:.2f}V')
        
    def update_ui(self):
        # 处理ROS回调
        rclpy.spin_once(self.node, timeout_sec=0)

def main():
    rclpy.init()
    node = Node('rover_control_ui')
    
    app = QApplication(sys.argv)
    window = RoverControlUI(node)
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 