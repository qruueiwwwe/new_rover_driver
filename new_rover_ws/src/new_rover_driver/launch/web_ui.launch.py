from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('new_rover_driver')
    web_ui_dir = os.path.join(pkg_dir, 'new_rover_driver', 'web_ui')
    
    return LaunchDescription([
        # 启动rosbridge服务器
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),
        
        # 启动机器人驱动节点
        Node(
            package='new_rover_driver',
            executable='rover_driver.py',
            name='rover_driver',
            output='screen'
        ),
        
        # 启动Web服务器
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080'],
            cwd=web_ui_dir,
            output='screen'
        ),
        
        # 自动打开浏览器（可选）
        ExecuteProcess(
            cmd=['python3', '-c', 'import webbrowser; webbrowser.open("http://localhost:8080")'],
            output='screen'
        )
    ]) 