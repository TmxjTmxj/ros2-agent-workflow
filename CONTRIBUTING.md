# Contributing

感谢你考虑为 `ros2-agent-workflow` 贡献代码、文档、测试或案例。

## 项目结构

```text
agent_ros/                 # 核心框架：profiles / discovery / safety / runtime / adapters
mcp_server/                # FastMCP 控制面
profiles/                  # 机器人 Profile 与任务 Profile
examples/hospital_delivery/# 医院送药参考案例
tests/                     # 根 Python 测试
docs/                      # 架构、对比、开发复盘
assets/                    # 图表与演示图片
```

## 环境准备

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

## 开发流程

1. 先建立可复现的失败测试（RED）
2. 最小修复（GREEN）
3. 运行根测试

```bash
.venv/bin/python -m pytest tests/ -q
```

Run the reproducible local quality gates before opening a pull request:

```bash
make check
```

4. 运行医院 ROS 案例测试前，确保已经 source ROS2 环境：

```bash
source /opt/ros/lyrical/setup.bash
/usr/bin/python3 -m pytest examples/hospital_delivery/tests/ -q
```

## 代码风格

- 保持 Python 3.11+ 兼容
- 所有对外边界都返回稳定错误码，不泄漏内部路径或异常
- 新 MCP 工具必须有类型 schema、超时、注解和测试
- 新适配器必须实现 `RobotAdapter` 契约，并遵守安全平面约束
- 测试文件保持确定性，不得依赖真实时间、网络或进程残留

## 提交信息

提交信息请按以下格式之一：

```text
feat: 新增 ...
fix: 修复 ...
docs: 更新 ...
test: 补强 ...
refactor: 重构 ...
design: 图表或前端资源调整
```

## 安全约束

- 不得通过 MCP 暴露 shell、任意 topic、任意 payload、文件路径或进程 PID
- 不得允许 Agent 直接发布 `/cmd_vel`
- 新增机器人与任务 Profile 必须通过 `jsonschema` 和 `tests/test_profiles.py`
- 真机适配必须说明物理急停绑定方案和硬件 operator challenge
- 不得在仓库中提交日志、私密凭据、个人路径、真实姓名或联系信息

## License

本项目使用 MIT License。提交代码即表示你同意该授权。
