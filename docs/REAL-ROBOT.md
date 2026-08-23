# 真机适配指南：Twist / Nav2

本仓库当前只在 Gazebo 高保真仿真中完成医院案例验证，但核心控制链
`MCP → SafetyGateway → RuntimeController → ROS2 Adapter` 与真机同构。
迁移到真实机器人时，保留安全层不变，替换机器人 Profile、Adapter 和
传感器验收项。

## 1. 明确边界

真机部署必须满足：

- 运动控制由单写入者节点或 Nav2 action server 完成
- Agent 不直接发布 `/cmd_vel`
- 物理急停必须绑定到 `SafetyGateway`
- 硬件模式必须通过 operator challenge 才能 `arm`
- 所有验收继续使用独立采样，不信任控制器自报

## 2. 差速底盘：TwistAdapter

### 2.1 机器人 Profile

```yaml
# profiles/robots/my-diff-robot.yaml
name: my-diff-robot
mode: hardware
namespace: /my_robot
frames:
  base: base_link
  odom: odom
adapter:
  kind: twist
interfaces:
  command:
    topic: /my_robot/cmd_vel
    type: geometry_msgs/msg/Twist
  odometry:
    topic: /my_robot/odom
    type: nav_msgs/msg/Odometry
limits:
  max_linear_velocity: 0.30
  max_angular_velocity: 1.20
  max_linear_acceleration: 0.60
  max_angular_acceleration: 1.50
safety:
  heartbeat_timeout: 1.0
  estop_topic: /my_robot/emergency_stop
observation_sources:
  - odometry
```

### 2.2 任务 Profile

```yaml
# profiles/tasks/my-diff-task.yaml
name: my-diff-task
robot_profile: my-diff-robot
stages:
  - name: go-to-door
    goal:
      frame: odom
      x: 2.0
      y: 0.0
      yaw: 0.0
    tolerance: 0.15
    timeout: 60.0
required_sensors:
  - odometry
evidence:
  sources:
    - odometry
recovery_policy: cancel_and_stop
```

### 2.3 硬件急停与 operator challenge

物理急停按钮应接入 ROS2 topic，例如 `std_msgs/msg/Bool`。在适配器层把
该按钮输入绑定到 `bind_physical_estop(handler)`，按钮按下时调用
`SafetyGateway.observe_physical_estop(True)`。

硬件 `arm` 前需要生成一次性 operator challenge：

```bash
.venv/bin/python -m agent_ros.cli --json hardware-challenge my-diff-robot
```

MCP 侧 `arm_robot` 默认 `dry_run=true`，只有硬件模式且携带 challenge
时才真正授权。

## 3. 导航机器人：Nav2Adapter

### 3.1 机器人 Profile

```yaml
# profiles/robots/my-nav2-robot.yaml
name: my-nav2-robot
mode: hardware
namespace: /my_nav_robot
frames:
  base: base_link
  odom: odom
adapter:
  kind: nav2
interfaces:
  navigation:
    action: /navigate_to_pose
    type: nav2_msgs/action/NavigateToPose
limits:
  max_linear_velocity: 0.30
  max_angular_velocity: 1.20
  max_linear_acceleration: 0.60
  max_angular_acceleration: 1.50
safety:
  heartbeat_timeout: 1.0
  estop_topic: /my_nav_robot/emergency_stop
observation_sources:
  - odometry
```

### 3.2 任务 Profile

```yaml
# profiles/tasks/my-nav2-task.yaml
name: my-nav2-task
robot_profile: my-nav2-robot
stages:
  - name: dock-to-kitchen
    goal:
      frame: map
      x: 3.0
      y: -1.5
      yaw: 0.0
    tolerance: 0.20
    timeout: 120.0
required_sensors:
  - odometry
evidence:
  sources:
    - odometry
recovery_policy: cancel_and_stop
```

### 3.3 工作方式

- `run_task` 会把阶段目标映射为 `NavigateToPose` action goal
- 取消任务会先取消 action，再执行零速安全停止
- Nav2 必须配置正确的 TF、地图和局部规划器
- 真实机器人上建议保留独立验收器，至少采样终点误差、停止漂移和
  `/cmd_vel` 发布者身份

## 4. 通用验证流程

1. 静态检查

```bash
.venv/bin/python -m pytest tests/test_profiles.py tests/test_adapters.py tests/test_runtime_controller.py -q
```

2. Profile 校验

```bash
.venv/bin/python -m agent_ros.cli --json verify-profile my-diff-robot
.venv/bin/python -m agent_ros.cli --json verify-profile my-nav2-robot
```

3. 真机空载测试

- 不接任务，先确认 `discover_robot` 能看到正确的 topic/action
- 空载执行 `arm` 和 `run_task`，确认控制器与急停链路
- 用 `emergency_stop` 测试安全停车
- 通过 `task_status` 确认状态机闭环
- 用 `observe` 和独立验收器生成证据

## 5. 迁移医院案例到真机的注意点

- 医院案例适配器 `hospital_delivery` 被限制为仿真封闭生命周期，不要直接
  迁移到真机
- 真机请使用 `twist` 或 `nav2` 适配器
- 路点坐标、容差、时间预算需要按真实场地重新标定
- 相机、激光、接触传感器应按新机器人重新配置并纳入独立验收
- 物理急停和 operator challenge 是硬件模式的强制项
