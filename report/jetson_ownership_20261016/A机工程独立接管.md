# A机由当前工程独立接管（2026-10-16）

用户明确要求：旧工程不能影响当前工程，A机必须由drone_payload_catch主导。已撤销“运行时直接复用旧工程目录和旧服务”的方式，改为独立受控运行环境及本工程服务。旧工程只保留为回退资料。

## 当前运行关系

| 项目 | 接管后的状态 |
| --- | --- |
| 当前工程版本 | 13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a |
| previous回退版本 | 8accbc1eca8dcba99b747f953e33ffddee2ac55f16d09db4d48a6df77a4781dd |
| Agent | payload-catch-agent.service，active/enabled，UDP8888、域0 |
| 中继 | payload-catch-mav-relay.service，active/enabled，UDP14550 |
| 旧microxrce-agent.service | inactive/masked |
| 旧mav-relay.service | inactive/masked |
| Agent程序与库 | /opt/payload_catch/managed_20261016/bin和lib，独立复制、记录指纹并由root保护 |
| PX4消息 | /opt/payload_catch/managed_20261016/px4_msgs_ws，固定392e831提交，从纯Humble环境原生构建，无旧环境链/远程origin |
| 中继实现 | 当前工程tools/mavlink_relay.py，安装至受保护runtime目录，不引用旧工程脚本 |
| 交互环境 | 原域42导出停用，自动加载当前工程运行环境，域0；不启动任务 |

系统Humble和Jetson平台驱动作为系统基础设施使用。Agent所需/usr/local库已独立复制到受保护目录，运行时明确采用该目录，避免依赖旧工程的构建/安装目录。原工程、原服务文件、原profile和.bashrc均未销毁。

## 干扰排除

新profile要求旧服务必须inactive/masked；若被重新启用，检查会拒绝继续。新服务与旧服务设Conflicts，旧服务mask阻止原入口自动或普通手动启动。正式启动沿用角色锁、双机图/指纹门禁，并检查重复Agent、端口归属、环境/消息/程序指纹和其他ROS控制发布者。

runtime_env清除继承的旧Python/ROS/虚拟环境路径，再加载当前独立消息环境。核对交互shell实际域0及消息模块来源，无/home/nvidia/ros2_ws或Multi-UAV-simulation路径。对常见systemd、cron、桌面启动和profile入口扫描，未发现旧工程目录引用；未发现旧工程运行进程。

新中继仅接受配置的FC和GCS源IP，拒绝其他来源，不再任意学习新的地面站IP。它不是完整网络认证；只检查MAVLink起始字节，飞控负责内容/命令校验。默认GCS仍192.168.1.123，实际QGC电脑地址和地面站通路尚未验收。

## 安全迁移与验证

切换前订阅真实状态，确认SYSID1、未解锁且已落地；准备root保护的旧状态备份，设置180秒独立systemd自动回退，随后停止/停用/屏蔽旧服务、安装并启动新通信服务，切换发布链接和profile。新链路真实检查通过并再次确认地面后提交，回退定时器已取消。

独立px4_msgs构建首次SSH等待180秒超时，原唯一构建继续运行；记录PID14470并等待其结束，没有重复启动编译。事件日志确认包构建rc=0；随后完成独立环境路径检查与root封存。构建日志保留。

本地quick37/37通过；目标ARM构建及10环境、8中继、16部署组网、35释放、24安全、10起飞测试通过。新环境下完整机载隔离模拟约27.74秒通过，B、动力学、1.5秒机构及PWM仍为模拟。

首次接管后审计发现中继在地面站离线时因UDP发送超时重启。已停止该新中继并修复为非阻塞收发，发送队列满或网络不可达时丢弃当前包并计数，不重试、不延后重放命令。修复前文件/配置另存relay_before_nonblocking_fix；只替换本工程中继和发布版本，Agent保持运行，旧服务持续屏蔽。新增离线/队列/网络错误回归通过；修复后连续观察45.5秒，PID和NRestarts未变。这是本次现场稳定性证据，不代表长期可靠性或QGC通路已验收。

真实切换后及用户换电重启后检查均通过：状态/姿态/位置/电池新鲜且源时间递增，SYSID1、未解锁，没有发现ROS控制发布者。无真实飞行任务、解锁、电机或舵机动作；真实舵机配置禁用/未标定。没有残留本任务构建/模拟进程。未修改飞控参数、引脚和已核对启动文件。

用户换电使Jetson重启：旧boot=64244966-0f68-4732-b814-1da1248c28e9，新boot=95cbe7b2-836e-4add-9e46-9c2b5aa6ea7c。此前33脚临时PWM模式失效，重启后只读核对pad33=0x54、32c0000.pwm未请求/未使能且period/duty为0，sysfs没有导出的pwm0，当前为原始GPIO状态，没有输出脉冲；本轮未重新prepare或发脉冲。旧PWM基线跨boot失效，需要下一次明确的硬件操作及重新核验后才能再次开启，不能直接更换boot ID绕过保护。两项新通信服务已实际通过开机自启验收，旧服务重启后仍inactive/masked，飞行任务未自启。

## 开机行为与使用

上电后只有本工程管理的通信服务自动启动，飞行任务不自动启动、PWM不自动输出。旧通信服务被屏蔽。地面只读检查：

```bash
bash /home/nvidia/payload_catch_deploy/current/src/deploy/check_jetson.sh
```

它不生成起飞批准。双机真实自检、至少40%电量、定位/RC/几何标定及释放反馈等门禁保持。

## 回退

备份在/var/lib/payload_catch/ownership_20261016。确需撤销接管，确认地面停机后运行：

```bash
sudo -n python3 /opt/payload_catch/managed_20261016/manage_ownership.py restore --ground-confirmed
```

脚本要求当前版本仍等于本次接管版本，并实际订阅确认落地/未解锁，持任务/部署锁，恢复旧服务、原发布链接、profile和.bashrc。它会重新启用旧通信服务，明确用于故障恢复。本轮验证了自动回退的就绪与取消状态、备份和目标；没有刻意中断正常链路去实际演练回退。

## 证据

managed_check.json、final_audit.json、startup_audit.json、interactive_shell.json、ownership_state.log、ownership_switch.log、relay_fix_apply.log、relay_stability.log、relay_stability_before_reboot.log、managed_relay_journal.log、onboard_validation.log、jetson_result.json、jetson_mission.log、inspect.json、after_reboot.log、after.json、validation.json。after.log保留首次重启探针因沿用“通道必须已导出”旧假设产生的断言；after_reboot.log用硬件只读状态确认未导出的通道确实未输出，没有为通过检查而导出PWM。修改前源码在before_source/，首版接管证据在first_managed_version/；系统原文件保留于root备份目录。远端诊断资料在/home/nvidia/payload_catch_ownership_20261016。

旧工程的已知自动入口、目录依赖和默认环境已隔离。日后若人为重新运行其他控制程序或修改系统管理配置，仍须重新验收控制权；本次不宣称对任意未来root操作进行隔离。
