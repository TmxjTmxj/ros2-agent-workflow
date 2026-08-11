---
name: ros2-agent-workflow
description: 用Agent/CLI操控ROS2与Gazebo仿真。用户提ROS2、Gazebo、机器人仿真、turtlesim、ros_gz_bridge时使用。环境为ros-lyrical(官方源安装)，含MCP Server与完整工作流脚本。
version: 1.0.0
author: hermes-secretary
license: MIT
platforms: [linux]
metadata:
  hermes:
    tags: [ros2, gazebo, robotics, mcp, simulation]
---

# ROS2 Agent 工作流（lyrical）

## 触发条件
- 用户提及 ROS2 / Gazebo / 机器人仿真 / turtlesim / ros_gz_bridge
- 用户要"操控ROS2""让agent连接仿真"

## 环境信息（Claude Code 安装，已实测验证）
- **发行版**: ros-lyrical（ROS2 2026 新版本，`/opt/ros/lyrical`）
- **安装方式**: 官方源 `ros-lyrical-desktop-full` + `ros-lyrical-ros-gz`（Claude Code 完成）
- **Gazebo**: gz sim 10.4.0（`/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz`）
- **桥接**: ros_gz_bridge（parameter_bridge）
- **python3.14** 与 rclpy 可能不兼容 → 优先用 CLI（ros2/gz），不用 python API
- **无显示器**: 设 `QT_QPA_PLATFORM=offscreen`（headless 模式）

## 加载环境
```bash
source /opt/ros/lyrical/setup.bash   # 或 source scripts/ros2_env.sh
export PATH="$PATH:/opt/ros/lyrical/opt/gz_tools_vendor/bin"
```

## 常用命令速查
```bash
# 节点/话题
ros2 node list
ros2 topic list -t
ros2 topic echo /turtle1/pose --once
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"

# Gazebo（headless）
gz sim -s -r empty.sdf &
gz topic -l
gz model --list

# 桥接 ROS2 ↔ Gazebo
ros2 run ros_gz_bridge parameter_bridge \
  /world/empty/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V
```

## 工作流（MCP Server 提供工具）
`mcp_server/ros2_mcp_server.py` — FastMCP 实现的 ROS2 桥接层：
- list_nodes / list_topics / topic_echo / pub_topic / run_ros2_cmd
- gz_sim_launch / gz_topic_list / gz_topic_echo / gz_model_list
- turtle_launch / turtle_spawn / turtle_teleport

启动：
```bash
cd ~/ros2-agent-workflow
source scripts/ros2_env.sh
pip install fastmcp
python3 mcp_server/ros2_mcp_server.py
```

## 演示脚本（已实测通过）
- `scripts/demo_turtlesim.sh` — turtlesim 完整闭环（启动→查询→控制→反馈→清理）
- `scripts/demo_gazebo_bridge.sh` — Gazebo↔ROS2 桥接闭环

## 坑
- turtlesim 是 Qt GUI，无 DISPLAY 时必须 `QT_QPA_PLATFORM=offscreen`
- rclpy 在 python3.14 有兼容问题 → 别用 `import rclpy`，用 CLI
- gazebo 启动需 5-6 秒才出话题，查询前先 sleep
- 清理演示进程用精确 PID（勿 pkill -f，会误杀）
