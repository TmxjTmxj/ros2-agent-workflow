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

## 环境信息
- **发行版**: ros-lyrical（`/opt/ros/lyrical`）
- **仿真器**: gz sim 10.4.0
- **桥接**: ros_gz_bridge（parameter_bridge，医院案例配置在
  `examples/hospital_delivery/config/ros_gz_bridge.yaml`）
- **Python**: 项目 `.venv` 使用 Python 3.14；原生图探测使用 ROS apt 提供的
  `rclpy`/EmPy，MCP 子进程只继承固定 ROS overlay 环境变量
- **无显示器**: `hospital_delivery.launch.py` 默认 `headless:=true`

## 加载环境
```bash
source /opt/ros/lyrical/setup.bash
cd <ros2-agent-workflow>
python3 -m venv .venv
.venv/bin/pip install -e .
```

## 通用 MCP 控制面（当前版本）
`mcp_server/ros2_mcp_server.py` 只暴露有界任务级工具，不暴露 shell 或任意
ROS 名称/载荷：

- `discover_robot` / `validate_profile` / `connection_status` / `list_capabilities`
- `arm_robot` / `run_task` / `task_status` / `cancel_task`
- `emergency_stop` / `observe` / `get_evidence` / `stop_runtime`

安全链路：MCP → RuntimeController/SafetyGateway → 持久 `rclpy` adapter →
ROS2 topics/services → `ros_gz_bridge` → Gazebo。运动控制由单写入者
`mission_controller` 完成，Agent 不直接发 `/cmd_vel`。

## 医院送药案例
```bash
# 直接 ROS2 控制 + 独立验收
bash scripts/demo_hospital.sh --headless --verify

# 生产 FastMCP stdio 固定工具序列
source /opt/ros/lyrical/setup.bash
.venv/bin/python examples/hospital_delivery/scripts/run_via_mcp.py
```

成功轨迹写入 `examples/hospital_delivery/evidence/mcp_agent_trace.json`；
独立验收报告在 `examples/hospital_delivery/evidence/acceptance_report.json`。

## 终态证据屏障
任务 `succeeded` 时先冻结终点里程计快照，再停止运动并清理 ROS/Gazebo；
`observe` 只读不可变快照，避免“成功取证 vs 进程清理”竞态。

## 坑
- 每次启动前确认没有旧 `gz-sim-main` / `parameter_bridge` /
  `mission_controller`；双 Gazebo 实例会导致 odom 不可信
- 赛题计时用 ROS `/clock` 仿真时间；墙钟只做失联/慢宿主机 watchdog
- 清理进程用项目生命周期脚本的精确 PID/PGID 校验，不要宽泛 `pkill -f`
- 顶层 `.runtime/` 保存安全审计，异常断电后可能进入 fail-closed
  quarantine；确认无进程后可整体归档该本地运行时目录再重跑
