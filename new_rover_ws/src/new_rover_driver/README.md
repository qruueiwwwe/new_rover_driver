# New Rover Driver

这是一个基于ROS2的机器人驱动包，提供了基本的机器人控制功能。

## 功能特点

- 基本的机器人运动控制
- 键盘控制接口
- LED控制
- 速度限制和安全保护

## 依赖项

- ROS2 Foxy或更高版本
- Python 3.8或更高版本
- 必要的ROS2消息包（geometry_msgs, std_msgs等）

## 安装

1. 将包克隆到你的ROS2工作空间：
```bash
cd ~/ros2_ws/src
git clone [your-repository-url]
```

2. 构建工作空间：
```bash
cd ~/ros2_ws
colcon build --packages-select new_rover_driver
```

3. 设置环境：
```bash
source ~/ros2_ws/install/setup.bash
```

## 使用方法

### 启动驱动节点

```bash
ros2 run new_rover_driver rover_driver
```

### 启动键盘控制

```bash
ros2 run new_rover_driver keyboard_control
```

### 键盘控制说明

- W: 前进
- S: 后退
- A: 左转
- D: 右转
- Q: 退出程序

## 话题

### 订阅的话题

- `/cmd_vel` (geometry_msgs/Twist): 速度命令

### 发布的话题

- `/cmd_vel` (geometry_msgs/Twist): 处理后的速度命令
- `/led_status` (std_msgs/Bool): LED状态

## 参数

- `max_linear_speed`: 最大线速度 (默认: 2.0 m/s)
- `max_angular_speed`: 最大角速度 (默认: 1.0 rad/s)
- `wheel_base`: 轮距 (默认: 0.3 m)
- `wheel_radius`: 轮子半径 (默认: 0.05 m)

## 注意事项

1. 确保在使用前正确设置机器人参数
2. 建议在测试时使用较低的速度值
3. 确保在紧急情况下能够及时停止机器人

## 许可证

[Your License] 