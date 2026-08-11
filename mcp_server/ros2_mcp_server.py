#!/usr/bin/env python3
"""
ROS2 MCP Server — 让 AI Agent 通过 MCP 协议操控 ROS2 / Gazebo
=============================================================
基于 FastMCP 实现的 ROS2 桥接层，提供以下工具：
  - list_nodes      列出当前所有 ROS2 节点
  - list_topics     列出所有话题
  - topic_echo      订阅并读取话题最新消息
  - pub_topic       发布消息到话题
  - run_ros2_cmd    执行任意 ros2 CLI 命令（安全白名单）
  - gz_sim_launch   启动 Gazebo 仿真（带指定世界）
  - gz_spawn_model  在 Gazebo 中生成模型
  - gz_topic_info   查询 Gazebo 话题

依赖：fastmcp, rclpy（ROS2 环境）
运行：source /opt/ros/lyrical/setup.bash && python3 mcp_server/ros2_mcp_server.py
"""

import os
import sys
import shutil
import subprocess
import json
import tempfile
from typing import Optional

try:
    from fastmcp import FastMCP
except ImportError:
    print("需要安装 fastmcp: pip install fastmcp", file=sys.stderr)
    sys.exit(1)

# ===== ROS2 环境准备 =====
ROS_ROOT = "/opt/ros/lyrical"
os.environ.setdefault("AMENT_PREFIX_PATH", ROS_ROOT)
os.environ.setdefault("ROS_DISTRO", "lyrical")

mcp = FastMCP("ros2-mcp-server")

# ===== 工具：ROS2 节点/话题查询 =====

def _ros2_env():
    """构造 ROS2 命令的环境变量"""
    env = os.environ.copy()
    env["AMENT_PREFIX_PATH"] = ROS_ROOT
    env["PYTHONPATH"] = f"{ROS_ROOT}/lib/python3.14/site-packages:{env.get('PYTHONPATH', '')}"
    env["LD_LIBRARY_PATH"] = f"{ROS_ROOT}/lib:{env.get('LD_LIBRARY_PATH', '')}"
    env["PATH"] = f"{ROS_ROOT}/bin:{env.get('PATH', '')}"
    return env

def _run(cmd, timeout=30):
    """执行命令，返回 stdout"""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout, env=_ros2_env())
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return f"[超时] {cmd}"
    except Exception as e:
        return f"[错误] {e}"


@mcp.tool()
def list_nodes() -> str:
    """列出当前所有 ROS2 节点"""
    return _run("ros2 node list")


@mcp.tool()
def list_topics() -> str:
    """列出所有 ROS2 话题（含类型）"""
    return _run("ros2 topic list -t")


@mcp.tool()
def topic_echo(topic: str, timeout: int = 5) -> str:
    """订阅并读取话题最新消息（timeout 秒内）"""
    return _run(f"timeout {timeout} ros2 topic echo {topic} --once")


@mcp.tool()
def topic_info(topic: str) -> str:
    """查询话题详细信息（类型、发布者、订阅者）"""
    return _run(f"ros2 topic info {topic}")


@mcp.tool()
def pub_topic(topic: str, msg_type: str, msg_value: str) -> str:
    """发布消息到话题。msg_type 如 std_msgs/msg/String, geometry_msgs/msg/Twist；
    msg_value 为 YAML 格式消息内容，如 '{data: hello}' 或 '{linear: {x: 0.5}, angular: {z: 0.1}}'"""
    return _run(f"ros2 topic pub --once {topic} {msg_type} '{msg_value}'")


@mcp.tool()
def run_ros2_cmd(cmd: str) -> str:
    """执行任意 ros2 CLI 命令（白名单：node/topic/service/param/action/interface/launch）。
    示例：'node list', 'topic list -t', 'service list', 'param list /node_name'"""
    # 安全白名单
    allowed_prefixes = ("node", "topic", "service", "param", "action",
                        "interface", "launch", "doctor", "daemon", "bag")
    first = cmd.strip().split()[0] if cmd.strip() else ""
    if first not in allowed_prefixes:
        return f"❌ 拒绝执行：白名单只允许 {allowed_prefixes}，收到 '{first}'"
    return _run(f"ros2 {cmd}")


# ===== 工具：Gazebo 仿真控制 =====

@mcp.tool()
def gz_sim_launch(world: str = "empty.sdf", headless: bool = False) -> str:
    """启动 Gazebo 仿真。world: 世界文件（默认 empty.sdf）；
    headless=True 时不启动 GUI（适合服务器/CI）"""
    gz_bin = f"{ROS_ROOT}/opt/gz_tools_vendor/bin/gz"
    flags = " -s -r" if headless else " -r"
    return _run(f"{gz_bin} sim{flags} {world} & echo 'Gazebo 已启动 (PID $!)'")


@mcp.tool()
def gz_topic_list() -> str:
    """列出 Gazebo 当前所有话题"""
    gz_bin = f"{ROS_ROOT}/opt/gz_tools_vendor/bin/gz"
    return _run(f"{gz_bin} topic -l")


@mcp.tool()
def gz_topic_echo(topic: str, timeout: int = 5) -> str:
    """读取 Gazebo 话题最新消息"""
    gz_bin = f"{ROS_ROOT}/opt/gz_tools_vendor/bin/gz"
    return _run(f"timeout {timeout} {gz_bin} topic -e {topic}")


@mcp.tool()
def gz_model_list() -> str:
    """列出 Gazebo 世界中的模型"""
    gz_bin = f"{ROS_ROOT}/opt/gz_tools_vendor/bin/gz"
    return _run(f"{gz_bin} model --list")


# ===== 工具：Turtlesim 演示 =====

@mcp.tool()
def turtle_launch() -> str:
    """启动 turtlesim 演示节点（经典 ROS2 入门 demo）"""
    return _run("ros2 run turtlesim turtlesim_node & echo 'turtlesim 已启动 (PID $!)'")


@mcp.tool()
def turtle_spawn(x: float = 5.0, y: float = 5.0, name: str = "turtle2") -> str:
    """在 turtlesim 中生成第二只乌龟"""
    return _run(f"ros2 service call /spawn turtlesim/srv/Spawn "
                f"\"{{x: {x}, y: {y}, theta: 0.0, name: '{name}'}}\"")


@mcp.tool()
def turtle_teleport(x: float = 5.0, y: float = 5.0, name: str = "turtle1") -> str:
    """将乌龟瞬移到指定坐标"""
    return _run(f"ros2 topic pub --once /{name}/teleport_absolute "
                f"turtlesim/msg/Pose \"{{x: {x}, y: {y}, theta: 0.0}}\"")


# ===== 主入口 =====
if __name__ == "__main__":
    print("🤖 ROS2 MCP Server 启动中...")
    print(f"  ROS 环境: {ROS_ROOT}")
    print(f"  Gazebo: {'可用' if shutil.which('gz') or os.path.exists(f'{ROS_ROOT}/opt/gz_tools_vendor/bin/gz') else '不可用'}")
    print("  工具: list_nodes / list_topics / topic_echo / pub_topic / run_ros2_cmd / gz_* / turtle_*")
    mcp.run()
