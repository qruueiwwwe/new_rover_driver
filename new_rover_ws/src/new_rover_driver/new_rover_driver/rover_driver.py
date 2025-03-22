#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Imu, MagneticField, JointState
from std_msgs.msg import Bool, Int32, Float32
from std_srvs.srv import SetBool
import cv2
import numpy as np
import math
import threading
import time
import random

class RoverDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Initialize parameters
        self.declare_parameter('min_distance', 0.5)  # Minimum safety distance
        self.declare_parameter('max_distance', 3.0)  # Maximum detection distance
        
        # Get parameter values
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        
        # Initialize state variables
        self.is_running = False
        self.current_velocity = Twist()
        self.latest_scan = None
        self.latest_rgb = None
        self.latest_imu = None
        self.latest_mag = None
        self.battery_voltage = 0.0
        self.car_version = 1.0
        self.current_steer_angle = 0.0
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.battery_pub = self.create_publisher(Float32, 'voltage', 100)
        self.version_pub = self.create_publisher(Float32, 'edition', 100)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 100)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 100)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 100)
        self.buzzer_pub = self.create_publisher(Bool, 'Buzzer', 1)
        self.rgb_light_pub = self.create_publisher(Int32, 'RGBLight', 10)
        
        # Create subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.rgb_sub = self.create_subscription(Image, 'camera/rgb/image_raw', self.rgb_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)
        self.voice_cmd_sub = self.create_subscription(Int32, 'voice_command', self.voice_command_callback, 10)
        
        # Create services
        self.start_srv = self.create_service(SetBool, 'start_driver', self.start_driver_callback)
        self.stop_srv = self.create_service(SetBool, 'stop_driver', self.stop_driver_callback)
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Create thread lock
        self.lock = threading.Lock()
        
        self.get_logger().info('RoverDriver node initialization completed')

    def timer_callback(self):
        """Timer callback function for publishing status information"""
        if not self.is_running:
            return
            
        # Publish battery voltage
        battery_msg = Float32()
        battery_msg.data = self.battery_voltage
        self.battery_pub.publish(battery_msg)
        
        # Publish version information
        version_msg = Float32()
        version_msg.data = self.car_version
        self.version_pub.publish(version_msg)
        
        # Publish IMU data
        if self.latest_imu:
            self.imu_pub.publish(self.latest_imu)
            
        # Publish magnetometer data
        if self.latest_mag:
            self.mag_pub.publish(self.latest_mag)
            
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = Clock().now().to_msg()
        joint_state.header.frame_id = "joint_states"
        joint_state.name = ["back_right_joint", "back_left_joint", 
                          "front_left_steer_joint", "front_left_wheel_joint",
                          "front_right_steer_joint", "front_right_wheel_joint"]
        
        # Calculate steering angle (radians)
        steer_radis = self.current_steer_angle * math.pi / 180.0
        joint_state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
        
        # Add random wheel positions if robot is moving
        if not (self.current_velocity.linear.x == 0 and 
                self.current_velocity.angular.z == 0):
            i = random.uniform(-math.pi, math.pi)
            joint_state.position = [i, i, steer_radis, i, steer_radis, i]
            
        self.joint_state_pub.publish(joint_state)

    def scan_callback(self, msg):
        """LiDAR data callback"""
        with self.lock:
            self.latest_scan = msg
            # Check for obstacles
            if self.is_running:
                self.check_obstacles()

    def rgb_callback(self, msg):
        """RGB camera data callback"""
        with self.lock:
            self.latest_rgb = msg
            # Process RGB image
            if self.is_running:
                self.process_rgb_image()

    def imu_callback(self, msg):
        """IMU data callback"""
        with self.lock:
            self.latest_imu = msg

    def mag_callback(self, msg):
        """Magnetometer data callback"""
        with self.lock:
            self.latest_mag = msg

    def voice_command_callback(self, msg):
        """Voice command callback"""
        if not self.is_running:
            return
            
        if msg.data == 2 or msg.data == 0:  # Stop
            self.stop()
        elif msg.data == 4:  # Forward
            self.move_forward(0.5)
            time.sleep(5)
            self.stop()
        elif msg.data == 5:  # Backward
            self.move_backward(0.5)
            time.sleep(5)
            self.stop()
        elif msg.data == 6:  # Turn left
            self.rotate(0.2)
            time.sleep(5)
            self.stop()
        elif msg.data == 7:  # Turn right
            self.rotate(-0.2)
            time.sleep(5)
            self.stop()
        elif msg.data == 11:  # Red light
            self.set_rgb_light(255, 0, 0)
        elif msg.data == 12:  # Green light
            self.set_rgb_light(0, 255, 0)
        elif msg.data == 13:  # Blue light
            self.set_rgb_light(0, 0, 255)
        elif msg.data == 14:  # Yellow light
            self.set_rgb_light(255, 255, 0)
        elif msg.data == 15:  # Flow effect
            self.set_rgb_effect(1)
        elif msg.data == 16:  # Gradient effect
            self.set_rgb_effect(4)
        elif msg.data == 17:  # Breathing effect
            self.set_rgb_effect(3)
        elif msg.data == 18:  # Show battery level
            self.set_rgb_effect(6)

    def start_driver_callback(self, request, response):
        """Start driver service callback"""
        if request.data:
            self.is_running = True
            response.success = True
            response.message = "Driver started"
        else:
            self.is_running = False
            response.success = True
            response.message = "Driver stopped"
        return response

    def stop_driver_callback(self, request, response):
        """Stop driver service callback"""
        if request.data:
            self.is_running = False
            self.stop()
            response.success = True
            response.message = "Driver stopped"
        return response

    def check_obstacles(self):
        """Check for obstacles"""
        if not self.latest_scan:
            return
            
        # Check front obstacles
        front_ranges = self.latest_scan.ranges[len(self.latest_scan.ranges)//3:2*len(self.latest_scan.ranges)//3]
        min_distance = min(front_ranges)
        
        if min_distance < self.min_distance:
            self.stop()
            self.get_logger().warn(f'Obstacle detected in front, distance: {min_distance:.2f}m')

    def process_rgb_image(self):
        """Process RGB image"""
        if not self.latest_rgb:
            return
            
        try:
            # Convert image format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
            
            # Image enhancement and processing
            # 1. Convert to HSV color space for better color processing
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 2. Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
            
            # 3. Edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # 4. Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 5. Draw main contours
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
            
            # 6. Add text information
            cv2.putText(cv_image,
                       f'Objects: {len(contours)}',
                       (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       1,
                       (0, 255, 0),
                       2)
            
            # 7. Issue warning if large objects are detected
            large_objects = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
            if large_objects:
                self.get_logger().info(f'Detected {len(large_objects)} large objects')
                
                # Check for objects in center region
                height, width = cv_image.shape[:2]
                center_region = edges[height//3:2*height//3, width//3:2*width//3]
                if np.any(center_region):
                    self.get_logger().warn('Object detected in front!')
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            processed_msg.header = self.latest_rgb.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def move_forward(self, speed=0.5):
        """Move forward"""
        cmd = Twist()
        cmd.linear.x = speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def move_backward(self, speed=0.5):
        """Move backward"""
        cmd = Twist()
        cmd.linear.x = -speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def rotate(self, angular_speed):
        """Rotate"""
        cmd = Twist()
        cmd.angular.z = angular_speed
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)
        
    def stop(self):
        """Stop"""
        cmd = Twist()
        self.current_velocity = cmd
        self.cmd_vel_pub.publish(cmd)

    def set_rgb_light(self, r, g, b):
        """Set RGB light color"""
        msg = Int32()
        msg.data = (r << 16) | (g << 8) | b
        self.rgb_light_pub.publish(msg)

    def set_rgb_effect(self, effect):
        """Set RGB light effect"""
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