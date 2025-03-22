#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import webbrowser

class WebUIServer(Node):
    def __init__(self):
        super().__init__('web_ui_server')
        self.server = None
        self.start_server()
        
    def start_server(self):
        # 切换到包含index.html的目录
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        
        # 创建HTTP服务器
        self.server = HTTPServer(('localhost', 8080), SimpleHTTPRequestHandler)
        
        # 在新线程中启动服务器
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        
        self.get_logger().info('Web UI服务器已启动在 http://localhost:8080')
        
        # 自动打开浏览器
        webbrowser.open('http://localhost:8080')
        
    def stop_server(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()

def main():
    rclpy.init()
    server = WebUIServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop_server()
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 