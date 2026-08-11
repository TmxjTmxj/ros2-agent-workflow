#!/usr/bin/env bash
# ros2_env.sh — ROS2 lyrical 环境加载器
# 用法: source ros2_env.sh  （然后即可使用 ros2/gz 命令）
# 由 Claude Code 通过官方源安装: ros-lyrical-desktop-full + ros-lyrical-ros-gz

export ROS_DISTRO=lyrical
export ROS_ROOT=/opt/ros/lyrical
export AMENT_PREFIX_PATH="$ROS_ROOT"
export PYTHONPATH="$ROS_ROOT/lib/python3.14/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ROS_ROOT/lib:${LD_LIBRARY_PATH:-}"
export PATH="$ROS_ROOT/bin:$ROS_ROOT/opt/gz_tools_vendor/bin:${PATH}"

# Gazebo GUI 在无显示器环境用 offscreen
if [ -z "$DISPLAY" ]; then
    export QT_QPA_PLATFORM=offscreen
fi

# 验证
if [ -x "$ROS_ROOT/bin/ros2" ]; then
    echo "✅ ROS2 ($ROS_DISTRO) 就绪: $(ros2 --help 2>&1 | head -1)"
else
    echo "❌ ROS2 未找到，请检查安装: $ROS_ROOT/bin/ros2"
fi
