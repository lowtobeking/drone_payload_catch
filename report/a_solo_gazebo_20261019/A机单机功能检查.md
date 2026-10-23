# A 机单机 Gazebo 功能检查（2026-10-19）

本次在 Ubuntu-22.04 电脑内运行真实 PX4 1.16 SITL 和 Gazebo GUI，使用工程的 A-only 监督及任务节点。没有连接或部署 Jetson，没有真实飞控解锁、PWM 动作或 B 机。

## 已通过

- 完整流程连续通过两轮：自检 → 等待显式 start → Offboard 预发送 → 模式确认 → 解锁 → 垂直起飞 1 m → 稳定悬停 10 s → 请求降落 → 着陆并上锁。
- 通过条件同时检查 PX4 估计高度、Gazebo 真实模型高度、HOVER 阶段和正常 LAND_REQUESTED；保护结束不会被计作正常任务成功。
- 隔离 ROS 8 场景通过：关键数据缺失拒飞、39% 电量拒飞、控制发布者冲突拒飞、仅检查模式无控制输出、正常起降、操作员停止后降落、任务进程失联后降落、人工接管后不抢回。
- A-only 逻辑 26 项、起飞门禁 10 项、MAVLink 参数读取 7 项和飞控参数校验全部通过；git diff --check 通过。

| 运行 | PX4 估计最大升高 | Gazebo 模型最高 z | 最终状态 |
| --- | --- | --- | --- |
| run_103809 | 1.012 m | 0.997 m | landed=true，armed=false |
| run_104115 | 1.017 m | 1.009 m | landed=true，armed=false |

仿真提供模拟遥控器输入，以及从 Gazebo 模型姿态生成的外部定位；两者均进入 PX4 处理，不伪造 PX4 输出健康遥测。无 GPS 的本轮通过，不能替代真实定位和遥控器验收。X500 仿真参数不能直接复制到实机。

## 修正范围

1. 参数读取跳过飞控转发的 GCS 心跳，但仍验证 A 的 SYSID1/component1/PX4 身份。
2. 识别 PX4 参数列表中的 _HASH_CHECK 元数据，继续检查完整参数计数和索引。
3. 兼容 PX4 1.16 的 BAT_LOW_THR/BAT_CRIT_THR/BAT_EMERGEN_THR，仍拒绝冲突阈值。
4. 单机适配器按实际 1 Hz 着陆状态设置接收窗口。
5. 仅隔离 Gazebo 测试注入仿真时钟，处理跨传输通道到达顺序；真实节点默认时钟和门禁保持原逻辑。

早期运行出现过解锁后电池与磁力计遥测中断，任务执行了保护降落，没有误报成功。声音模块对照未解决该现象，相关参数已撤回；根因尚未最终定位。之后两个连续运行完整通过，因此本次证明正常功能及上述故障分支可执行，尚不能保证长期运行稳定性。

舵机 1700 µs/约 1.5 s 的投放、双机通信与接取不属于这个 A-only 起降入口；本次没有验收这些功能，也没有放宽正式双机门禁。

## 证据及复现

- functional_validation.json：本次汇总。
- run_103809/result.json、run_104115/result.json：完整任务结果。
- 各运行目录的 px4.log、mission.log、supervisor.log 和 log/*.ulg：原始证据。
- ../a_solo_validation_20261019/ros_summary.json：8 个隔离 ROS 场景。
- diagnostic_helpers/：排障临时脚本归档，不是日常启动入口。

复现（仅本机隔离仿真）：

```bash
cd /home/caolihao/drone_payload_catch
sudo -n /usr/bin/python3 tools/run_a_solo_gazebo.py
```

启动器建立仅 loopback 的网络命名空间，普通用户运行仿真，结束后清理本次进程。两次成功运行以及本轮 ROS 回归的工作进程均已退出，完整主机进程清单保存在 process_audit.txt。
