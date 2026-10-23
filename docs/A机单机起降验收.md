# A 机单机起降验收

本入口验证 A 自检、Offboard/解锁确认、垂直起飞、连续稳定悬停、自动降落和落地上锁。无需 B，不启动双机任务节点、释放门禁或 PWM 驱动，不能用于单机带物块投放。正式 A/B 流程仍使用原来的双机门禁。

## 当前交付与限制

实现和验证位于 WSL Ubuntu-22.04 的 /home/caolihao/drone_payload_catch。2026-10-19 已完成软件验收，尚未部署这个新增入口到 Jetson，也未进行真实起飞、解锁或 PWM 动作。隔离模拟使用真实 ROS 节点与合成 PX4 消息，模拟运动学；不等同于 PX4 SITL、动力学、HIL 或真实飞行验收。

上次实机记录：室内无桨、无有效水平定位、无遥控器，ROS 电池 cell_count=1 与 BAT1_N_CELLS=4/总电压不一致。这些条件仍会阻止起飞；当前状态需再次实机读取确认。

## 入口与默认任务

- 配置模板：config/a_solo.example.yaml，必须填写实际 hostname/板卡 serial、确认空载和场地检查。默认两个确认项为 false，主机身份为 FILL_ME。
- 节点：a_solo_validation 的 supervisor 与 mission 两个独立进程；独立的 /a_validation/ 通道。没有 B 的状态订阅/控制，也不给 /payload/ 释放授权。
- 起飞前：真实 A 遥测、完整实时飞控参数、本机通信服务环境检查，连续健康 3 秒；电量至少 40%，PX4 自检通过，姿态、位置/速度/航向有效、无关键估计器/飞控故障、独立 RC 有效、着陆且上锁。
- 额外核验：A SYSID1/component1、传感器源时间/新鲜度、实际配置的电池节数及总电压一致、无竞争控制发布者。
- 默认：NED 原地上升 1m，升速上限 0.3m/s，水平纠偏上限 0.2m/s；误差及速度均在阈值内连续 2 秒，再连续稳定悬停 10 秒。随后请求 PX4 AUTO_LAND，要求命令接受、降落模式以及新鲜“已着陆且已上锁”观测才能完成。
- 本地验收边界默认半径 2m、高度 2m、倾角 25°；20 秒未达到稳定悬停或总任务超过 60 秒，取消任务并请求降落。它们是伴随计算机监督阈值，不是已实测的制动距离保证。
- 飞控参数同样检查 RC 接管/切动力开关、RC/Offboard 失联降落、围栏、起降速度、落地自动上锁及 CBRK_FLIGHTTERM=0。只读，不自动修改参数或复用旧工程的推力标定值。

positioning=outdoor_gps 需要有效 GPS 和匹配 EKF 配置；external_local 需要真实有效的动捕/视觉定位与匹配 EKF 配置，并且仍强制 RC、局部位置和其他健康条件。不能靠切换此项把无定位状态变成可飞。

## 部署完成后，在 A 的 SSH 会话中使用

以下是未来部署到 A 后的使用顺序；本轮没有执行这些实机命令。不要在室内无桨状态发 start。

使用已审核的同一发布包部署、构建并核对 current 后，打开两个使用同一运行环境的 SSH 会话：

```bash
source /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
```

在发布目录之外保留实际配置，例如 /home/nvidia/payload_catch_deploy/a_solo.yaml；从模板复制并逐项填写，禁止直接修改发布包内文件后忽略校验错误。源代码变更、重启或换配置需要重新检查。

先生成实时自检记录（MAVLink 参数端点必须是已核对的、专用于本次读取的 A 机端点，不得抢占现有 relay 端口）：

```bash
python3 /home/nvidia/payload_catch_deploy/current/src/tools/a_solo_preflight.py \
  --config /home/nvidia/payload_catch_deploy/a_solo.yaml \
  --host-profile /home/nvidia/payload_catch_deploy/current/src/config/jetson_a.runtime.yaml \
  --mavlink udpout:127.0.0.1:18570 \
  --out /home/nvidia/payload_catch_deploy/a_solo_preflight.json
```

18570 仅为格式示例，须先核实实际 relay 的读端点。参数编码由 AUTOPILOT_VERSION 能力位确认；PX4 整数参数按 bytewise 类型解析，编码不明或参数列表不完整则拒绝。自检失败立即覆盖旧通过记录，并保留错误报告。通过记录 60 秒有效，绑定本机启动身份、配置和代码摘要；不能被双机门禁接受，不是安全认证令牌。

先用 check 模式看阻塞原因（不会发控制消息；即使自检通过也不能启动）：

```bash
bash /home/nvidia/payload_catch_deploy/current/src/deploy/run_a_solo.sh \
  /home/nvidia/payload_catch_deploy/a_solo.yaml \
  /home/nvidia/payload_catch_deploy/a_solo_preflight.json \
  /home/nvidia/payload_catch_deploy/current/src/config/jetson_a.runtime.yaml check
```

需要进入起降验收时，现场条件确认后停止 check，重新做自检，将末尾 check 改为 execute 启动。execute 仍不会自动起飞，等待显式 start。该脚本与双机入口共用 mission.lock，校验 A 角色/current/构建/发布包完整性和项目自有通信服务，不改服务、不设置上电自启。

另一 SSH 会话查看状态并显式启动：

```bash
python3 /home/nvidia/payload_catch_deploy/current/src/tools/a_solo_control.py status
python3 /home/nvidia/payload_catch_deploy/current/src/tools/a_solo_control.py start --confirm A_ONLY_TAKEOFF
```

状态必须为 READY 且 execute=true；start 时重查自检有效性、传感器、着陆/上锁、任务心跳和竞争控制程序。成功后先连续发送 2 秒零速度预流，再切 Offboard，确认 ACK/模式后才请求解锁，确认 ACK/模式/真实已解锁后才执行上升。不自动重试整轮任务，不自动恢复起飞。

## 操作员停止与故障

```bash
# 取消任务；已解锁或解锁结果不明时请求降落
python3 /home/nvidia/payload_catch_deploy/current/src/tools/a_solo_control.py stop

# 仅紧急切动力，可能使空中无人机坠落；与正常停止分开
python3 /home/nvidia/payload_catch_deploy/current/src/tools/a_solo_control.py kill --confirm CUT_MOTOR_POWER
```

stop、超时、越界、估计器坐标重置、导航/RC/电池严重故障、控制竞争、任务心跳超过 0.5 秒不新鲜均锁存本轮取消；有解锁风险则监督器独立请求降落。起飞后电量低于 40% 不会单独触发急降，沿用运行时电池严重故障规则与飞控低电保护。

RC/PX4 已切走 Offboard 时，停止任务输出和自动模式重试，标记 MANUAL，交给实际接管者；不会自动重新解锁。LAND 期间也尊重人工模式。落地/上锁没有得到新鲜反馈或命令确认时标记失败，不用计时伪造成功。

SSH 请求必须能到达监督器才能生效。整个 Jetson 断电/卡死、通信链路失效时依赖已核验的 PX4 Offboard/RC 失联配置和独立 RC；SSH 不能替代独立紧急接管链路。无可靠定位时软件能请求降落，但不能承诺降落精度或实际成功。

## 软件验证证据

report/a_solo_validation_20261019/：

- unit.log：26 项逻辑/门禁测试。
- ros_summary.json、各场景 JSON 和日志：8 个隔离 ROS 场景；正常单机起降、操作员停止、任务进程失联、人工接管，以及缺关键数据、低电、控制冲突、只检查均不动作。无 B，无真实飞控，无 PWM。
- dual_gate_regression.log：原双机起飞门禁 10 项。
- safety_regression.log：原安全运行 24 项。
- release_regression.log：无反馈定时释放 10 项。
- build.log / launch_args.log：本地 Humble 构建及新 launch 参数解析。
- validation.json：源码摘要、结果汇总及本轮测试进程退出检查。

模拟为了缩短测试，将目标高度设为 0.5m、悬停 2 秒；交付默认配置仍为 1m/10秒。尚需真实参数/传感器整改验收、无桨只读检查、定位和 RC/失联保护验收，再决定实际首飞。
