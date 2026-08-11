#!/usr/bin/env bash
# demo_turtlesim.sh — ROS2 Agent 工作流演示（无头模式）
# 演示内容: 启动 turtlesim → 列出节点/话题 → 发布速度指令 → 读取位姿 → 清理
# 用法: bash demo_turtlesim.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros2_env.sh"

echo ""
echo "════════════════════════════════════════"
echo "  🤖 ROS2 Agent 工作流演示 (turtlesim)"
echo "════════════════════════════════════════"

echo ""
echo "── 1. 启动 turtlesim 节点 ──"
ros2 run turtlesim turtlesim_node > /tmp/turtlesim_demo.log 2>&1 &
TURTLE_PID=$!
echo "   turtlesim 已启动 (PID $TURTLE_PID)"
sleep 3

echo ""
echo "── 2. 列出所有节点 ──"
ros2 node list

echo ""
echo "── 3. 列出所有话题 ──"
ros2 topic list

echo ""
echo "── 4. 发布速度指令让乌龟转圈 ──"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" 2>&1 | tail -1
echo "   ✅ 速度指令已发布 (v=2.0 m/s, ω=1.8 rad/s)"

echo ""
echo "── 5. 读取乌龟当前位置 ──"
timeout 5 ros2 topic echo /turtle1/pose --once 2>&1 | grep -E "x:|y:|theta:" | head -3

echo ""
echo "── 6. 清理 ──"
kill $TURTLE_PID 2>/dev/null || true
echo "   ✅ 演示完成，turtlesim 已停止"
echo ""
echo "════════════════════════════════════════"
echo "  ✅ 完整闭环: 启动 → 查询 → 控制 → 反馈"
echo "════════════════════════════════════════"
