# 🤖 ROS2 Agent 工作流

**用 AI Agent 操控 ROS2 与 Gazebo 仿真 —— MCP Server + 一键演示脚本 + 完整技能文档**

![ROS2](https://img.shields.io/badge/ROS2-lyrical-orange) ![Gazebo](https://img.shields.io/badge/Gazebo-gz_sim_10.4-blue) ![MCP](https://img.shields.io/badge/MCP-FastMCP-green)

> 本项目展示了如何让 AI Agent 通过 **MCP 协议** 连接并操控 **ROS2（lyrical 发行版）** 与 **Gazebo 仿真**，实现"启动仿真 → 查询状态 → 发布指令 → 读取反馈"的完整闭环。环境由 Claude Code 通过官方源安装，本项目在其上构建了 Agent 可用的桥接层。

> **硬件安全边界：** 本项目是研究与教学软件，不能替代认证的工业安全系统。真机 profile 只有在独立 emergency channel 经预检后才可验证；`emergency_stop` 返回仅表示非阻塞 zero/disable 命令已入队，不保证物理执行。Hospital runner 仅支持仿真，不能用于真机急停。运行时状态会将尚未验证的真机通道标记为 `unverified`。

---

## 🏗️ 架构

```
┌────────────────────────────────────────────────────┐
│                 AI Agent (Claude Code / Hermes)     │
│                      │  MCP 协议                    │
│                      ▼                              │
│           ┌──────────────────────┐                  │
│           │  ros2_mcp_server.py  │  ← FastMCP 桥接层 │
│           │  (list/pub/gz/turtle)│                  │
│           └──────────┬───────────┘                  │
│                      │ CLI                          │
│           ┌──────────▼───────────┐                  │
│           │      ROS2 lyrical    │                  │
│           │  /opt/ros/lyrical    │                  │
│           └──────────┬───────────┘                  │
│                      │ ros_gz_bridge                │
│           ┌──────────▼───────────┐                  │
│           │   Gazebo gz sim      │                  │
│           │   10.4.0 (headless)  │                  │
│           └──────────────────────┘                  │
└────────────────────────────────────────────────────┘
```

## ✨ 功能

### MCP Server 工具（`mcp_server/ros2_mcp_server.py`）
| 工具 | 功能 |
|------|------|
| `list_nodes` | 列出所有 ROS2 节点 |
| `list_topics` | 列出所有话题（含类型） |
| `topic_echo` | 读取话题最新消息 |
| `pub_topic` | 发布消息到话题 |
| `run_ros2_cmd` | 执行 ROS2 CLI（白名单安全控制） |
| `gz_sim_launch` | 启动 Gazebo 仿真 |
| `gz_topic_list/echo` | 查询 Gazebo 话题 |
| `gz_model_list` | 列出仿真世界模型 |
| `turtle_launch/spawn/teleport` | turtlesim 演示控制 |

### 一键演示脚本（`scripts/`）
- **`demo_turtlesim.sh`** — turtlesim 完整闭环：启动 → 列节点 → 列话题 → 发布速度 → 读位姿 → 清理
- **`demo_gazebo_bridge.sh`** — Gazebo ↔ ROS2 桥接：启动 gz sim → 查话题 → 起 bridge → 验证互通

## 🚀 快速开始

```bash
# 1. 加载 ROS2 环境
source scripts/ros2_env.sh

# 2. 跑 turtlesim 演示（无需显示器）
bash scripts/demo_turtlesim.sh

# 3. 跑 Gazebo 桥接演示
bash scripts/demo_gazebo_bridge.sh

# 4. 启动 MCP Server（供 AI Agent 连接）
pip install fastmcp
python3 mcp_server/ros2_mcp_server.py
```

## ✅ 实测验证（2026-08-11）

### turtlesim 闭环
```
/turtlesim                    ← 节点启动
/turtle1/cmd_vel              ← 话题就绪
发布 Twist (v=2.0, ω=1.8)     ← 控制指令
x: 6.602 y: 6.938 θ: 1.814    ← 位姿反馈（乌龟动了）
```

### Gazebo 桥接闭环
```
Gazebo 话题: /world/empty/pose/info, /clock, /stats
ros_gz_bridge: parameter_bridge 启动
ROS2 侧出现: /world/empty/pose/info, /clock  ← 桥接成功！
```

## 📁 目录结构

```
ros2-agent-workflow/
├── mcp_server/
│   └── ros2_mcp_server.py     # FastMCP ROS2 桥接层
├── scripts/
│   ├── ros2_env.sh            # 环境加载器
│   ├── demo_turtlesim.sh      # turtlesim 演示
│   └── demo_gazebo_bridge.sh  # Gazebo 桥接演示
├── skills/
│   └── ros2-agent-workflow.md # Agent 技能文档
├── launch/
└── docs/
```

## ⚙️ 环境要求

- Ubuntu + ROS2 **lyrical**（`ros-lyrical-desktop-full` + `ros-lyrical-ros-gz`）
- Gazebo gz sim ≥ 10.0
- Python 3.10+（MCP Server 用）
- 无显示器环境需 `QT_QPA_PLATFORM=offscreen`

## 📄 License

MIT
