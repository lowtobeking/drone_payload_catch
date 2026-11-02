> **2026-10-16 本机旧参数场景评估**：用户确认drone_package_20260908曾部署在当前无人机。核对真实params快照后发现0.30/0.50悬停推力、3/12m/s限速等版本差异，不把配置脚本建议当已写实机值。当前B规划a_brake=6m/s²而旧MPC_ACC_UP_MAX约1.95/4，需按实际固件及B接物后能力重验规划，不能照抄或宣称6已实现。建议先静止投放、任务分阶段限速、分别验证A减重/B增重响应，下洗及桨盘通道也需考虑。4S历史不能代替16.44V/1芯当前遥测来源排查。分析见docs/旧工程电机控制参考.md补充；本轮仅文档，无SSH/代码或参数更改/硬件动作。

> **2026-10-16 用户指定旧工程电机控制参考**：只读检查Ubuntu24.04的drone_package_20260908/Multi-UAV-simulation-full。旧mpc_node使用PX4 Offboard / TrajectorySetpoint，并非单电机RPM直控；当前px4_iface同类控制链，fcu_params已有部分旧工程限速/推力关系建议。旧机悬停推力、约0.5秒速度滞后及降落/失联时长不能直接套用A/B。记录docs/旧工程电机控制参考.md；本轮仅文档与记忆，无任务代码/飞控参数变更、无SSH或电机动作。

> **2026-10-16 最新：用户无锁扣传感器，改用定时动作模式**：confirmation_mode=timed，授权后直出1700µs，从实际写入计时约1.5秒后先停输出再发布ACTION_COMPLETE；physically_confirmed=false/detached_at=null，不生成物理released Bool。A验计划/会话/指纹后清场并标记release_unconfirmed，B只记录estimated_release_at，接取仍需真实下落观测；sensor模式保留。无锁扣反馈不再阻止timed readiness，其他40%/RC/双机/自检/一次性/急停/看门狗不变。enabled/calibrated仍false，未部署或动作硬件。quick39/39、10新增回归、3隔离ROS定时场景及原sensor/launch通过。见docs/无反馈定时释放模式.md、report/timed_release_20261016。覆盖旧“必须锁扣真实反馈”的约定；无反馈无法自动检测卡扣/卡物。

> **2026-10-16 最新：第一步固定A机地面入口完成**：工程tools/run_a_ground_live.py/.sh，A机独立入口/home/nvidia/payload_catch_deploy/ground/runtime_20261016/run_a_ground_live.sh，加载current真实节点/遥测，A角色/完整性/落地未解锁/源时间及新鲜度检查，输出重映射、auto_arm=false、/dev/null无批准、PWM禁用，保留全部飞行门禁。机载20.05秒PID7308运行通过，消息0、清理复查通过，两通信服务保持0重启；quick38/38、4遥测故障测试通过。current及接管回退未变。每次机载ground/reports独立日志。下一步先查电池16.44V/1芯不一致，暂不动作舵机或飞行。见docs/A机地面检查与后续验收.md及report/a_ground_entry_20261016。

> **2026-10-16 最新：A机真实节点地面启动通过**：SSH在Jetson运行已部署ANode/SafetySupervisor/ReleaseGuard/PayloadServo约20.02秒（PID6807），真实遥测、auto_arm=false、缺批准/require_fleet_link门禁保留，飞控全部输出重映射诊断话题且实际消息0，PWM disabled/无输出。真实飞控始终未解锁，结束后无任务进程/控制发布者。不是完整fleet launch或飞行：B/RC/有效水平定位/现场fleet配置/真实反馈与标定/双机参数批准缺失。电池约91%、16.44V但1芯不一致；通用预检models目录与真机包范围差异待修。日志report/a_task_readiness_20261016。所有辅助文件今后只放Ubuntu22.04工程内，旧Windows资料已移至report/workfiles/windows_home_20261016。

> **2026-10-16 最新：A机以本工程独立主导**：current=13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a、previous=8accbc…；旧microxrce-agent/mav-relay均inactive/masked，新payload-catch-agent与payload-catch-mav-relay active/enabled。程序/Agent库/独立px4_msgs在/opt/payload_catch/managed_20261016且root保护，不再运行依赖旧工程或/home/nvidia/ros2_ws；旧bashrc域42停用，交互域0。迁移使用真实落地/未解锁保护与180秒自动回退，验收后已提交取消定时器，备份/人工回退保留。真实通信与启动/进程/环境审计通过，quick37/37和机载10+8+16+35+24+10项、27.74秒隔离模拟通过；无实机解锁/PWM、无任务自启或残留。中继离线发送超时已修复为非阻塞丢包，重启后连续45秒无重启。用户换电已重启Jetson，新boot95cbe7b2…；33脚临时PWM模式失效，现pad0x54、控制器未使能/未导出，未重新开启；旧PWM基线须重新核验。两项通信服务自启验收通过，飞控仍未解锁。旧工程仅保留回退。详见report/jetson_ownership_20261016/A机工程独立接管.md，覆盖此前直接引用旧工程服务/消息的方案。

> **2026-10-16 最新：复用A机现有通信基础设施**：已部署current=8accbc1eca8dcba99b747f953e33ffddee2ac55f16d09db4d48a6df77a4781dd、previous=fa46032…；host/runtime.yaml绑定现有Humble/px4_msgs@392e831、Agent8888域0、中继14550和实测网络，不改服务/飞控参数/自启，PID1003/1002保持。新增只读deploy/check_jetson.sh、host_runtime和启动前核验，run_fleet自动采用域0并核对完整fleet配置一致；fleet域0需domain_zero_confirmed=true，B/几何/全部起飞门禁保留。真实域42→0读取FC状态/姿态/位置/电池通过，quick36/36与机载9+16+35+24+10项、27.89秒隔离任务模拟通过。无实机解锁/PWM/后台测试残留。可回退版本和绑定，见report/jetson_reuse_20261016/A机现有服务复用验收.md及deploy/README_jetson_reuse.md。

> **2026-10-16 最新：已在A机部署并运行完整隔离模拟**：current=fa46032caae12d28676da9014545e6b888a944572744ad205e50fcb0812c026a、previous=e27cb2…；Jetson ARM构建及35释放/24安全/10起飞测试通过，分进程完整模拟约27.89秒通过并清理退出。机载入口tools/run_a_full_mission_isolated.py兼容版本包，独立netns/普通用户/MockPWM，边沿为理想命令+1.5秒模型，反馈延迟另记；未放宽安全阈值。真实飞控未解锁，PWM仍20ms/duty0/enable0，原服务与已核对boot文件不变，真实舵机禁用/未标定。正常降落释放误报修复已部署。详见report/a_onboard_sim_20261016/A机部署与机载模拟.md。

> **2026-10-16 A机完整流程隔离模拟通过**：Ubuntu-22.04独立netns/普通用户MockPWM，真实任务节点完成自检→模拟起飞→直出1700µs→约1.493秒模拟脱离→清场→先B后A模拟落地，约28.08秒；缺数据/39%电量拒飞，连续健康3秒门禁保留。修复已完成释放被正常降落停止通知误报FAULT，新增回归保留开锁中止和原故障锁存/单次保护。释放35项、quick35/35、完整流程和diff检查通过。无真实飞控指令/PWM输出，未部署，A实机仍e27cb2…；轻量运动学及合成B/GPS/RC/反馈不等于SITL或实机验收。证据report/a_full_mission_20261016/A机完整流程模拟.md。

> **2026-10-16 A机实机无桨短测**：无QGC，PX4 ACTUATOR_TEST逐电机1..4各8%/1秒，ACK/退出接受，用户确认四只均转停；始终未解锁。33脚独立重核本boot64244966…后临时PWM直出1700µs约0.8秒，用户确认转停；当前0x405/20ms/duty0/enable0。未部署新版本、未改飞控参数/启动文件/原通信服务。定时清理not-loaded已处理并核对无残留。报告report/a_ground_debug_20261016；物理方向/真实PWM波形/释放标定未验收，电池4S配置与1芯遥测来源待查。

> **2026-10-16 A 机地面联调优先**：A/B/guard 取消锁存、源时间/顺序/计划校验；B 从当前观测位置速度规划剩余接取并保留真实脱离时间；开锁全过程复核双机权限。新增独立 netns/普通用户 Mock 联调入口 tools/run_a_ground_isolated.py，正常/动作前中急停/B 就绪中断均通过，飞控输出为零。真实 PWM 仍禁用/未标定；未 SSH/重新部署，已部署 A 仍 e27cb2caa69a0ff8fc9cf4d43adbcc3a40419dc07c803536187a1a3608a9d9cb。详见 report/ab_optimization_20261016/A机联调与双机优化说明.md。

> **2026-10-15 最新时序对齐**：用户确认重新测量直出1700µs→脱离仍约1.5秒，配置operator_direct记录；双方T脱离/T-1.5秒直出。新增真实worker写入截止时间与时间回执，真实release_event带传感器边沿/会话/nonce/签名，B不再以Bool接收时间判释放。仍disabled/uncalibrated，真实反馈和时钟/HIL验收不变。验证report/timing_alignment_20261015；本轮无硬件动作。

> **2026-10-15 最新：直接1700µs释放**：用户取消缓变，飞行worker与地面GUI/agent首个输出即目标脉宽，限1200–1800、停止及看门狗保留。旧GUI缓变观测1.5秒不能当直出延迟；配置仍1.5历史参考但unmeasured_direct/disabled/uncalibrated。真实启用须本地直出→脱离重新测量、local_command_direct口径；双方按脱离时刻协调。仅软件，不SSH/部署/动作。见report/direct_release_20261015。

> **2026-10-15 当前投放延迟已接入**：连续舵机50Hz、1500→1700µs/100µs每秒缓变，1.5秒GUI点击→脱离初测估计。真实启动A/B提前量1.7秒，guard提前1.5秒执行，B保持对正等待真实下落；worker2秒上限/0.25秒失联保护，传感器确认后即停。仍disabled/uncalibrated/gui_apply；本地时序重新测量后才允许local_command标定。详见释放接入说明；本轮仅软件修改、无实机输出。

> **2026-10-15 当前实物与最新限幅**：用户确认360°连续旋转，1500µs附近中立，两侧控制方向/速度。地面GUI与远端输出固定1200–1800µs，移除扩展开关；无时长/心跳保护不变。仅软件修改，无新实机输出，飞行配置未改。实际波形与中立点待测；旧位置解释及宽范围属于历史。

> **2026-10-15 用户改回脉宽界面（最新）**：GUI直接输入整数µs，默认1500，默认1400–1600/确认扩展1000–2000；移除角度输入/换算/反馈，无输出时长，始终seconds=0持续保持；保留100µs/s缓变、停止/Esc/断线保护、只读基线门禁及硬件量化说明。飞行代码和远端agent未改，不新发实机脉冲。关闭旧窗口再启动。pwm_angle.py仅保留历史辅助/测试，当前GUI不导入。

> **2026-10-15 用户授权开启PWM模式（最新实机状态）**：已独立核对板卡、原pad0x54、通道未export、GPIO unused、启动哈希未变、无飞行任务并持独占锁；归档旧baseline/state/helper/manifest与前后快照到Jetson `/home/nvidia/pwm_pin33_safety/boot_recheck_841a584a-abd2-43d8-b005-b208a8a9c764_20261015T154611`，建立当前boot841a...有效基线。现33脚0x405，50Hz(period20000000)、duty0、enable0、normal；仅PWM模式保持，无舵机脉冲。真实上位机status/ping/close握手通过。恢复原GPIO用sudo -n python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py restore（先退出占用GUI）。旧只读过期状态已解除；连续转圈根因未排除，此次未发送运动信号。

> **2026-10-15 重启后上位机只读连接修复**：最新日志明确Boot changed导致agent构造退出，SSH本身正常。现在板卡/boot哈希核对通过且33脚原值、通道未export时，旧boot基线允许只读连接，显示基线过期；prepare/output/restore三条写路径均远端拒绝，GUI亦拒绝发送；stop/close仅重新确认idle而不写入。旧baseline/temporary_state未变，不绕过写门禁。异常非原始引脚/已export通道仍拒绝接管。未输出PWM。

> **2026-10-15 实际PWM驱动精度确认**：用户要求现有电源/信号接线不改，不做回环；用户原装舵机、未注意1500µs表现。只读反汇编实际pwm-tegra.ko确认round(duty*256/period)量化：50Hz每档约78.125µs/标称7.03°，sysfs为请求值非实际精确波形；GUI新增说明并改请求值标签。1500请求估算约1484.375、1533约1562.5（按标称20ms，非实测）。量化不能解释连续转圈，根因未定；不提高频率、不新发脉冲、不改跨boot基线。旧程序/循环角度未发现。记录report/pwm_link_fix_20261015/continuous_rotation_diagnosis.md及installed_pwm_driver_disassembly.txt。输入回读工具已准备且7项离线测试通过，用户拒绝改接线，未实机运行。

> **2026-10-15 断电后已重连（最新实机状态）**：IP仍192.168.155.186；新boot_id=841a584a-abd2-43d8-b005-b208a8a9c764。只读核对33脚寄存器0x54（初始GPIO），32c0000.pwm通道未export，无PWM输出；extlinux.conf/DTB哈希与原manifest一致。旧基线及temporary_state.json仍记上次boot 588d2395-ab3e-4ec5-9eeb-497bf00967d4，属于过期记录，不能直接prepare/restore或只替换boot ID绕过核验。本轮未改远端文件/引脚，未更新基线、未发脉冲。原装ES08MD II连续转圈原因仍未明，用户无示波器/测试器；后续可考虑独立GPIO输入回读，须先断开舵机信号并核对接线。只读映射物理7=PAC.06/gpiochip0 offset144、29=PQ.05/105、31=PQ.06/106均显示unused input；尚未请求输入或确认其pad复用，不可直接视为已验收接线方案。

> **2026-10-15 紧急：ES08MD II实物连续转圈**：用户确认朝同一方向转超过一整圈，位置控制实物验证失败。事前只读sysfs为50Hz/1533µs/enable=1；已仅终止专用锁的台架agent PID8766，停脉冲、不回中、不恢复GPIO，用户确认已停下。再次只读核对period=20000000、duty=0、enable=0、pinmux=0x405、boot未变。不能从规格书推定这只实物的位置型/实际角度已验证；可能连续旋转版本/模式/改装、反馈故障或实际波形问题，原因未定。暂不再发运动指令；先核对实物标签/购买版本/改装及实际波形。问用户是否有360°/连续/可编程标记，待回复。证据report/pwm_link_fix_20261015/continuous_rotation_incident.json。

> **2026-10-15 用户PDF纠正ES08MD II角度**：已读取用户Downloads/YX65566-12g金属数字舵机ES08MD+II-200803.pdf第2页，原件副本docs/hardware/ES08MD_II_specification.pdf。1000–2000µs为总90°±10°，1500µs中立；900–2100µs为最大总110°±10°，并非180°。标称线性换算改1000/90µs每度，以中立为相对0°，正常±45°；实际位置精度/机构尚未标定。GUI默认软件±9°=1400–1600µs，确认后扩展±45°=1000–2000µs，后端限正常1000–2000µs，未用900/2100极值。PDF无频率参数，50Hz仍为地面起点。旧10µs/°暂定比例弃用。本轮未动作真实舵机，SSH仅只读和纯Mock协议测试。

> **2026-10-15 PWM断线修复**：状态过期不再禁止心跳；输出失联看门狗仍2秒，停脉冲后保持SSH等待恢复，30秒无有效心跳关闭空闲会话；恢复不自动输出，须手动再次应用。旧挑战/重复命令停脉冲并报告但不销毁会话，硬件错误仍退出。新增本地诊断日志（Windows %LOCALAPPDATA%/PWM33/logs）。当前角度依旧未标定：10µs/°仅暂定显示，±5/10是软件限幅而非舵机总行程，尚无180°脉宽映射依据；不得称角度准确或直接扩到机械端点。本轮仅只读SSH诊断，测试用Mock后端不触碰真实PWM。

> **2026-10-15 ES08MD角度保持更新**：用户将当前地面调试舵机更正为EMAX ES08MD II（用户已确认II版）。上位机改为相对估算角度，无输出时长、始终持续保持，停止/断线关闭脉冲并保留PWM模式。临时换算1500µs=相对0°、10µs/°（未标定），默认±5°=1450–1550µs，确认扩展±10°=1400–1600µs；50Hz为保守地面起点，官方频率/角度映射未确认。旧GDW的333Hz/1520µs只作历史验证，不可继承。连接不改频率，显式prepare仅在关闭输出时将旧333Hz通道改50Hz，后端独立限幅/心跳保护保留。本轮未SSH、未驱动新舵机、未修改飞行servo配置。21项模拟测试、工程quick 33/33、Windows窗口断言检查通过，证据report/pwm_angle_es08mdii_20261015。详见PWM33上位机使用说明.md。

> **2026-10-15 PWM33上位机**：新增Windows/Tkinter GUI、SSH临时远端控制、2秒心跳看门狗、缓变/限幅/定时与持续输出、停止保留PWM模式、显式恢复。见 `PWM33上位机使用说明.md`。用户目前要求保持33脚PWM复用(0x405)，空闲enable=0、duty=0；不可在测试结束自动恢复GPIO。Jetson本次boot已重新核验备份。1800µs请求被中断，尚未执行；GUI本轮仅模拟及真实只读握手，无新动作。

> **2026-10-15 双机部署代码**：新增 `catch_real_a/b_launch.py`、`fleet_runtime.py` / `fleet_link.py`、`tools/fleet_configure.py` / `fleet_probe.py` / `fleet_deploy.py`、`deploy/run_fleet.sh`。A 唯一安全监督/释放门禁/PWM，B 接取及 relnav；通过双方心跳与过期回执、代码+配置指纹、实际 SYSID A1/B2、图与发布者唯一性门禁。任务基类新增 require_fleet_link，起飞/任务/释放都检查。部署打包当前工作区，校验、分版本原机构建、显式激活/回退，不自动启动。root PWM 服务仅提供保护路径的模板，尚未安装。配置模板信息未填、PWM 未标定，不能真飞；本轮未连接/部署 Jetson。详见 `双机部署与通信说明.md`。

> **2026-10-14 PWM 释放链已接入**：后续以 `Jetson_PWM_释放逻辑接入说明.md` 为准。新增纯逻辑 `payload_servo.py`、ROS 驱动 `payload_servo_node.py`、临时 PWM 与独立心跳/回退 `servo_pwm.py`；真机启动及 B 门禁已对接。候选锁/开脉宽 1440/1600µs，延迟 0.08s 为模拟占位，默认 disabled/uncalibrated。没有实测标定与真实 `/payload/latch_feedback` 时禁止真实 PWM/起飞；PWM 写入不能冒充 `/payload/released`。命令前持久记录防止重启重复动作，停止不自动重新锁扣。所有本轮工作在 Ubuntu-22.04，未操作 Jetson。

> **2026-10-14 释放机构确认**：用户确认A机舵机打开锁扣，让物块自由下落，不是主动推动。物块脱离时继承A机运动状态；应标定舵机命令到真实脱离的延迟及变化范围，不能把PWM写入成功当成已经释放。锁住/释放脉宽尚未标定。

> **2026-10-14 用户开发约定与舵机用途**：后续开发仅在 Ubuntu-22.04 的本工程进行，Ubuntu-24.04 旧副本保留。A 机以舵机释放物块、B 机协调接取；已验证的 Jetson 物理33脚 PWM 是真实释放执行接口，需要接入现有 authorized_release 安全链。硬件驱动尚未集成，锁住/释放脉宽及脱离延迟尚待确认；空载测试幅度不能直接作为机构参数。详细约定见 AGENTS.md。

> **2026-10-14 Humble 迁移**：本副本位于 Ubuntu-22.04，新开发入口与验收记录见 [Ubuntu22.04_Humble_迁移与使用说明.md](Ubuntu22.04_Humble_迁移与使用说明.md)。下文 Jazzy 环境和历史 SITL 成绩属于原环境记录。

# MEMORY.md — 项目记忆（给下一个 AI / 未来的自己）

> **2026-10-13 新增强制起飞门禁**：`preflight_health.py` 是 CLI／节点共用的纯逻辑。
> 关键数据缺失、非有限值、源时间戳不递增、数据过期、PX4 自检未通过、任一电池 <40% 都拒飞。
> `preflight_check --live --param-mavlink-a ... --param-mavlink-b ... --write-approval ...`
> 完整通过后生成 60 秒检查记录；节点必须收到该记录，并连续 3 秒核验双机健康后才允许自动解锁。
> 真机、M1/M6 SITL 启动均强制自检；真机需要 `PARAM_MAVLINK_A/B`；直接 launch 要传 `preflight_report`。
> 旧自检跳过／可选说明已失效。室外 GPS 严格；室内须显式选择动捕策略。
> 测试新增 `tools/test_takeoff_gate.py`，无 ROS 离线执行；尚未验证 HIL／真机。
> 2026-10-13 后续安全修改已落实：独立 safety_supervisor、release_guard；停止锁存／显式双机恢复、命令回执与模式确认、B 先降落、HOLD/PULLBACK 清除任务前馈、RTK 源时间／质量门禁、真实捕获保持确认、RC 自检。
> 操作与限制见 `report/safety_followup_plan.md`；新增 tools/test_safety_runtime.py 在离线执行真实节点方法。硬件释放驱动尚未提供，必须改接 authorized_release 并台架/HIL 验收。历史 SITL 成绩不代表本轮验证。

> 最后更新：2026-10-09（新增离线自检套件）。**新对话请先读本文件**，再按需读 `README.md`、`report/`。
> 目标读者：接手本项目的 AI 助手。读完应能直接继续干活，不必重跑全部排查。

---

## 0. 一句话现状

「无人机 A 空投载荷、无人机 B 空中接住」项目。**离线算法层完整**（M0/M2/M3/M3+/M4 + 大量鲁棒性探索，含多组对照与负结果）；
**SITL 层跑通并已加固**（PX4-1.16 双机 + Gazebo 真实载荷），当前覆盖：

- **场景**：M1 水平会合；M6 垂直堆叠（离线 200/200；SITL 捕获→携带→双机分开落地）；
  **M6-moving 编队同速（支持速度 0.5/1.0 m/s，各 2/2 完美）**；
- **末端能力**：大漏斗 `x500_funnel_big`（捕获余量×14）、空心导向锥杯 `x500_funnel_cup`、
  **主动保持锁扣** `payload_lock`（接住→刚性携带→落地，绕开 `v_retain`）；
- **真机末端：圆形托盘**（塑料围边+泡棉缓冲；`M6_stack_tray*` 工况 + `tools/tray_sizing.py`
  选型器；6cm/100g 方块离线 MC≈99%；缓冲恢复系数 `e` 是生死线）；
- **真机末端托盘 SITL**：`models/x500_tray` + `models/payload_100g`（`FUNNEL_TYPE=tray`）；
  6cm/100g 方块 SITL **3/3**（捕获→携带→双机落地，无 failsafe，horiz 0.015–0.049m）；
- **托盘鲁棒/编队**：倾斜保持 **~40°**（>平盘/空心杯）；侧风 0→6m/s 98→93%；
  **编队同速需大托盘** `x500_tray_big`（内径40cm）→ 0.5/1.0 m/s 捕获+携带落地（`report/m6_tray_robustness.md`）；
- **托盘主动锁扣**：`FUNNEL_TYPE=tray PAYLOAD_LOCK=1`（`payload_lock_100g`）→ 捕获即锁定，
  **运动交接与盘径解耦**（小托盘30cm 编队0.5/1.0 均捕获+携带）；泡棉 **e 实测工具** `tools/foam_drop_test.py`
  + 虚拟落物台 `tools/gz_foam_drop_test.sh`（模型泡棉 e≈0）；
- **抗扰**：速度前馈/预测对正、增广风 KF、A 端迎风预补偿、自适应下潜；
- **协同**：释放握手（B 报就绪→A 释放权威→ack）、意图（预测落点）、时钟同步、编队握手、
  **及时释放 + 释放超时中止**；
- **安全**：不确定度 keep-out、释放后清场、**飞行终止(kill)** 监督；
- **控制/规划**：ZEM 终端导引、释放前落点余量闸；
- **安全约束链（优化 2–5）**：分级安全状态机（`OK/HOLD/PULLBACK/LAND/KILL` + 越界软限幅）、
  传感器/估计器约束（EKF `eph/epv` σ + 健康看门狗 + 限值）、姿态/角速率安全滤波（`report/opt_round2..5.md`）；
- **研究（协调方向）**：理论 **C1–C5 + T1–T3**（释放证书+最优性 / 协议时序界 / state-vs-intent /
  联合机动 / handover-CBF / 联合证书 / 延迟 CBF）；**SITL 逐项消融主表**；`MODE=full` 一键预设；
  报告 `report/coordination_*.md`、`coordination_theory.md`、`coordination_experiments.md`。

**未做**：真空心漏斗 / 真实夹爪/磁吸；真机；真实相对导航（现为真值+噪声）；平台 failsafe 边界（~4.9 m/s，
需修 B 的 EKF/磁罗盘）；协同研究的**理论保证（C1 概率证书）** 与真机验证。

---

## 1. 项目定义与硬约定

- **任务**：A 携载重物飞行，在算法求出的**释放时刻/会合点**抛投；B 实时机动到空间会合点接住。
  约束：**位置必须到达**（硬），**相对速度尽量小**（软）。
- **第一版简化（已实现）**：A 悬停释放；载荷在 Gazebo 里真实下落；捕获用**软件判据**
  `|p_B−p_p| < r_c` 且 `|v_B−v_p| < v_c`（暂无真实吸附机构）。
- **坐标**：一切用**世界系 NED**（x=北, y=东, z=下），高度=`-z`。
  Gazebo ENU→NED：`NED=[ENU_y, ENU_x, -ENU_z]`。
- **单一真值源**：所有几何/参数/工况都在 `config/catch_scenarios.yaml`，**不要硬编码**。
- **核心算法层不依赖 ROS**（`payload_model` / `rendezvous` / `sim_core` / `mpc_terminal` / `payload_filter`），
  保证可离线验证；ROS 节点只做接口。

---

## 2. 环境事实（🔴 最容易踩坑，务必先看）

**本机可用的 SITL 环境不在 `$HOME/PX4-Autopilot`，而在 `~/drone_package_20260908/`。**
那里有一份 2026-09-08 的**已跑通工作快照**，其 `SITL仿真调试记忆_20260908.md` 是权威说明。

| 项 | 正解 | 备注 |
|---|---|---|
| PX4 树 | `~/drone_package_20260908/PX4-Autopilot-1.16` | **不要用 `$HOME/PX4-Autopilot`(main)** |
| 为什么 | main 的 `x500` 外层模型 **IMU 无噪声** → 静止输出恒 0 → `DataValidator` 判 STALE → 永不许解锁 | 这是之前 `Gyro #0 fail: STALE` 的真正根因；我在 main 上白排查了很久 |
| px4_msgs | `~/drone_package_20260908/ros2_ws/src/px4_msgs` @ **tag v1.16.2** | 与固件 1.16 匹配；不匹配会"话题静默不可见" |
| Gazebo | 系统 Harmonic **8.13**（`/usr/bin/gz`） | 另有一套 ROS vendor 8.11，**不要混用**（会把运行时库拉成 8.11） |
| world | `~/drone_package_20260908/gz_overrides/worlds/default.sdf`（自建"全系统"） | 1.16 自带 default.sdf 不含 Imu/NavSat/Sensors 插件 + 球坐标，传感器不发 |
| RMW | `rmw_fastrtps_cpp` | 本机全局是 cyclonedds；MicroXRCEAgent 是 Fast-DDS，不匹配则看不到话题 |
| `SYS_HAS_MAG` | 1 | 0 时静态起飞无 yaw 基准，gnss pos 融合被拒，`xy_valid` 永 false |
| 话题版本化 | 1.16：`vehicle_status_v1`；`vehicle_local_position`/`vehicle_attitude`/`trajectory_setpoint`/`offboard_control_mode` **无后缀** | |

**一键环境**：`source ~/drone_payload_catch/env.sh`
（内部已指向 PX4-1.16 + v1.16.2 workspace + 全系统 world + acados + fastrtps）。

已在 `/opt/ros/jazzy` 装好 `ros_gz_bridge`、`ros_gz_sim`，且 **`gz.transport13` / `gz.msgs10` Python 绑定可用**
（`PayloadNode` 用它订阅 Gazebo 载荷状态）。

---

## 3. 目录 / 文件地图

```
drone_payload_catch/
├── MEMORY.md                  ← 本文件
├── README.md                  ← 设计/用法/路线图
├── env.sh                     ← source 它进入正确环境
├── pytest.ini                 ← pytest 配置（testpaths=tests）
├── tests/test_offline_suite.py← 把 tools/test_*.py 收进 pytest（`pytest -q`）
├── .github/workflows/checks.yml ← CI：push/PR 跑 quick 自检 + pytest
├── Makefile                   ← `make check` / `make check-quick` / `make test`
├── .githooks/pre-commit       ← 提交前自动跑秒级自检（`git config core.hooksPath .githooks`）
├── config/catch_scenarios.yaml← 单一真值源（defaults/layouts/scenarios/thresholds）
├── payload_catch/
│   ├── payload_model.py       ← 载荷抛体模型（解析；可选阻力/风）
│   ├── rendezvous.py          ← 会合规划：min-energy 三次、软终端速度、过冲惩罚、分段参考、solve_inflight
│   ├── sim_core.py            ← 离线闭环仿真（A 恒速飞行 + B 控制 + 捕获 + 闭环重规划）
│   ├── mpc_terminal.py        ← B 的 acados 终端 MPC（含指纹缓存）
│   ├── payload_filter.py      ← 载荷状态估计（KF / 朴素）
│   ├── relnav.py              ← **相对定位纯逻辑**：大地→NED/杆臂/原点无关相对化（自测）
│   ├── relnav_node.py         ← **相对定位驱动**（RTK/px4/sim → /drone_a/state）
│   ├── contact_detect.py      ← **接触检测**（加速度尖峰/速度反转/外部开关；自测）
│   ├── uncertainty.py         ← **相对不确定度模型**（公共抵消 ρ + 杆臂×姿态 + 残差；自测）
│   ├── dynamics.py            ← **四旋翼聚合约束**（倾角+推力；替代裸双积分器；自测）
│   ├── safety_logic.py        ← **安全层纯函数**（软围栏/姿态滤波/EKF 看门狗/分级状态机/电池→Land；从 px4_iface 抽出以便离线单测）
│   ├── fcu_params.py          ← **飞控安全参数纯逻辑**（解析/判据/分组预置：THR_MIN<THR_HOVER、失效保护、围栏、EKF 源、电池）
│   ├── safety_matrix.py       ← **故障注入矩阵场景规格与判定**（纯逻辑：SCENARIOS + verdict）
│   ├── telemetry.py           ← **遥测有效性纯逻辑**（冻结 STALE / 溢出 OVERFLOW / 沉默 SILENCE / 兜底 HOVER；分析 launch.log）
│   ├── impact.py              ← **接触冲击**（冲量/峰值力/可恢复性/带载推力余量/柔性接触；自测）
│   ├── perception.py          ← **视觉载荷感知**（相机 FOV 门控 + 距离相关误差 + 深度 + 丢帧；自测）
│   ├── stats.py               ← **统计工具**（Wilson CI + 配对 McNemar；自测）
│   ├── stack_drop.py          ← M6 垂直堆叠投放：解析规划 + 漏斗保持判据 + 离线仿真（纯 Python）
│   ├── px4_iface.py           ← PX4 接口基类（话题/QoS/ARM+OFFBOARD/setpoint/世界系偏移）
│   ├── a_node.py              ← A：起飞→悬停释放点（M6 额外广播 /drone_a/state 供相对定位）
│   ├── b_node.py              ← B：会合捕获 或 M6 垂直堆叠状态机（对正→释放→下潜）
│   └── payload_node.py        ← 载荷：Gazebo 生成/瞬移/状态发布（含解析兜底）
├── launch/catch_launch.py     ← M1 SITL 启动
├── launch/catch_stack_launch.py ← M6 垂直堆叠 SITL 启动
├── models/payload/            ← Gazebo 载荷模型（0.3kg 小方盒 + odometry 插件）
├── models/payload_attached/   ← M6-moving：带 DetachableJoint 的挂载型载荷（随 A 飞、分离继承速度）
├── models/x500_funnel/        ← M6：x500 + 顶部刚性捕获圆锥（PX4_GZ_MODEL_NAME 附着）
├── models/x500_tray/          ← 真机末端：x500 + 圆形托盘（围边+泡棉，FUNNEL_TYPE=tray；tools/gen_tray.py 生成）
├── models/payload_100g/       ← 托盘用载荷：6cm 立方体 / 100g
├── models/x500_tray_big/      ← 大托盘（内径40cm，运动交接用；TRAY_R_IN=0.20 生成）
├── models/payload_attached_100g/ ← 编队×托盘：100g 挂载型载荷（DetachableJoint）
├── models/payload_lock_100g/  ← 托盘主动锁扣：100g 锁扣型载荷（PAYLOAD_LOCK=1）
├── tools/
│   ├── offline_run.py         ← 离线体检 CLI（--all/--plot/--sweep-noise/--compare/--controller-compare/--mc）
│   ├── stack_run.py           ← M6 垂直堆叠投放 CLI（--sweep-dive/--sweep-gap/--sweep-wind/--mc/--lead）
│   ├── handoff_explore.py     ← 交接场景公式推导的数值验证 CLI（静态/动态/机械臂/增速度）
│   ├── geom_opt.py            ← 鲁棒几何网格搜索（干扰下最大化最坏情况余量）
│   ├── funnel_model.py        ← 末端机构解析对比（平顶盘/空心锥/主动保持）
│   ├── drag_reject.py         ← 抗阻力/风扰（二次阻力、阵风）× 估计器对比
│   ├── gen_funnel_cup.py      ← 生成空心导向锥杯模型（x500_funnel_cup/funnel_cup/funnel_flat）
│   ├── prebuild_mpc.py        ← 预热 acados MPC（消除 SITL 启动期编译尖峰）
│   ├── sweep_sitl_difficulty.sh ← SITL 难度扫描
│   ├── sweep_m6_sitl.sh       ← M6 SITL 难度扫描（10 档，输出 report/m6_sitl_results.md）
│   ├── mc_m6_sitl.sh          ← M6 SITL 蒙特卡洛（每档 N 次换种子，输出 report/m6_sitl_mc.md）
│   ├── run_checks.sh          ← **一键离线自检**（--quick / 全量；纯模块 + tools/test_*.py + acados + offline --all）
│   ├── test_coord_cert.py     ← C1 证书纯逻辑单测（Rice/阈值/单调性/MC）
│   ├── test_keepout.py        ← C5 handover-CBF 纯逻辑单测（投影可行性/不变性）
│   ├── test_safety_logic.py   ← 安全层纯逻辑单测（围栏/姿态/看门狗/状态机）
│   ├── test_config.py         ← yaml 单一真值源契约（layout 引用/几何/真机段）
│   ├── test_purity.py         ← 纯算法层“无 ROS 依赖”守卫
│   ├── test_compileall.py     ← 全仓 .py 语法编译守卫
│   ├── preflight_check.py     ← **起飞前自检**（离线/`--sitl`/`--live`/`--gps`/`--logs`：配置/依赖/PX4/RMW/ROS/EKF/日志门）
│   ├── test_preflight.py      ← 起飞前自检逻辑单测（假 env/文件系统）
│   ├── live_probe.py          ← **运行时持续探测**（EKF/failsafe/磁罗盘/IMU/GPS；纯逻辑 evaluate 可单测）
│   ├── test_live_probe.py     ← live_probe.evaluate 单测（合成 Snapshot）
│   ├── uplink_test.py         ← **offboard 上行链路验证**（发心跳看 offboard_control_signal_lost）
│   ├── test_uplink_test.py    ← uplink verdict 单测
│   ├── check_log_validity.py  ← **遥测有效性检查**（launch.log 的 STALE/OVERFLOW/SILENCE/HOVER）
│   ├── test_telemetry.py      ← 遥测有效性判据单测（合成日志）
│   ├── preflight_params.py    ← **飞控参数飞行前检查**（--file/--mavlink）
│   ├── fcu_configure.py       ← **飞控参数配置器**（分组预置，dry-run 默认）
│   ├── test_fcu_params.py     ← 飞控参数判据单测（解析/判据/预置）
│   ├── sitl_safety_matrix.py  ← **安全层故障注入 SITL 矩阵**（5 场景，PASS/FAIL+报告）
│   ├── test_safety_matrix.py  ← 矩阵判定逻辑单测（verdict/场景自洽）
│   ├── sitl_check.sh          ← M6 SITL 端到端验收（委托 run_m6_sitl.sh，按事件给退出码）
│   └── smoke_acados.py        ← acados MPC 链冒烟（codegen+编译+求解+耗时）
├── run_m1_sitl.sh             ← M1 一键 SITL（gz + 2×PX4 + agent + 节点；可传 CTRL/A_HOVER/...）
├── run_m6_sitl.sh            ← M6 一键 SITL（B 带漏斗模型、5m 起飞、A 正上方释放）
└── report/
    ├── env_bringup.md         ← 环境排查全记录（含我在 PX4 上做的改动与回滚清单）
    ├── survey_and_sim_report.md ← **方向综述 + 完整仿真报告**（新读者先看这个）
    ├── mission_overview.md    ← **方向总述：被控对象/控制算法/仿真图表**
    ├── handoff_exploration.md ← **交接深度探索**：静态/动态/机械臂/增速度的公式推导与边界
    ├── robustness_wind.md     ← **侧风干扰鲁棒性**：速度前馈+预测对正，侧风容忍 1→3.5 m/s
    ├── m6_robustness_opt.md   ← **鲁棒性优化 II**：KF 估计/自适应下潜/漏斗几何（几何主杠杆）
    ├── end_effector_bigfunnel.md ← **末端能力**：大漏斗0.30(离线余量×14 + SITL STACK CAPTURED)
    ├── robust_geometry_and_retention.md ← **鲁棒几何+末端机构**：风下免下潜规则/空心锥/主动保持
    ├── hover_first_control.md ← **悬停优先控制**：minimal_dive 落到 b_node，SITL a_dive=0 捕获
    ├── drag_rejection.md      ← **抗阻力/风扰**：二次阻力+阵风；A 端迎风预补偿(w8: 0→40/40)
    ├── hollow_funnel_cup.md   ← **空心导向锥杯**：建模+SITL（含负结果：敞口杯倾斜不如平盘）
    ├── active_retention.md    ← **主动保持锁扣**：离线成立(下击暴流100/100)；SITL 落地受阻(负结果)
    ├── coordination.md        ← **双机协调**：现状/缺口/改进清单(握手/意图/时钟/安全)；含负结果
    ├── coordination_handshake.md ← **协同释放握手**：B 报就绪→A 释放权威→ack；SITL 验证
    ├── coordination_validation.md ← **协同协议 SITL 验证**：不变量+安全间隔+无 failsafe（全部通过）
    ├── optimization_backlog.md ← **后续可优化项总表**（分层+优先级+负结果+Top3）
    ├── opt_round2.md          ← **优化第二轮**：安全状态机/加速度前馈/σ共享/释放提交取消/3Dkeepout/自适应下潜
    ├── opt_round3.md          ← **优化第三轮（安全硬化）**：安全指令绕过限幅/围栏软限幅/安全状态进日志
    ├── opt_round4.md          ← **优化第四轮（传感器约束）**：EKF σ(eph/epv)/健康看门狗/估计器限值
    ├── opt_round5.md          ← **优化第五轮（姿态约束）**：倾角/角速率→安全滤波(水平衰减+加速度限额)
    ├── research_roadmap.md    ← **研究路线图（协调方向）**：C1–C5 贡献 + 现状差距 + 实验/代码映射 + MPU
    ├── paper_outline.md       ← **论文骨架（协调方向）**：题目/摘要/贡献/形式化/实验/相关工作/真机/时间线
    ├── paper_tasks.md         ← **论文任务追踪（活文档）**：D1–D4 / 理论 / 实验 / 真机 H1–H7 / 写作 P1–P8 / 风险 / 变更日志
    ├── coordination_probability.md ← **C1 概率证书**：二维脱靶模型 + `P(capture)≥1−ε` + 离线验证
    ├── coordination_information.md ← **C3 信息**：state vs intent（延迟容忍；intent 保留证书）
    ├── coordination_protocol.md ← **C2 协议时序界**：取消窗口/设计表/丢包失效模式
    ├── coordination_maneuver.md ← **C4 联合机动**：A 速度/高度 × B 机动联合优化
    ├── coordination_cbf.md   ← **C5 handover-CBF**：速度级防碰不变集 + 交接门
    ├── coordination_theory.md ← **协调理论**：T1 释放域最优性 / T2 联合证书 / T3 延迟 CBF
    ├── coordination_experiments.md ← **实验主表**：逐项消融（应力下时延 3.3→0.72→0.42s）
    ├── coordination_benchmark.md ← **协同交接基准（阶段 0）**：bench_coord.py 生成（coord_mode×intent×σ×delay + Wilson CI）
    ├── planning_control_opt.md ← **规划/协调+控制优化**：ZEM 终端导引 + 释放前落点余量闸
    ├── safety_control_review.md ← **保护控制审查**：已有(限幅/keep-out/释放闸) vs 缺口(geofence/看门狗/abort/避碰)
    ├── safety_supervisor.md    ← **安全监督+飞行终止(kill)**：/safety/kill_a|b + 异常自动 kill；SITL 验证
    ├── companion_safety.md     ← **companion 安全网**：失联看门狗（冻结→LAND）+ 硬碰撞地板（→HOLD）；对标参考 safety_filter
    ├── safety_injection_matrix.md ← **安全层故障注入矩阵**：5 场景逐项诱发 + 零误触发基线，5/5 PASS
    ├── safety_overview.md      ← **安全总览**：飞行前/飞行中/验证 全清单 + 与参考工程对照 + 仍缺项
    ├── m6_moving_speed.md      ← **M6-moving 加速度**：编队控制优化(死推算参考)+速度边界(2.0✅/3.0❌)
    ├── m5_sitl_results.md     ← M5 难度扫描结果
    └── m6_sitl_results.md     ← M6 SITL 难度扫描结果（2026-09-17）
    └── m6_sitl_mc.md          ← M6 SITL 蒙特卡洛结果（2026-09-17）
```

训练/编译产物：acados 缓存在 `~/.cache/payload_catch/acados_terminal_mpc/`（**非 /tmp**）。

---

## 4. 怎么跑

### 4.1 离线（不需要 ROS/SITL，秒级）
```bash
cd ~/drone_payload_catch
python3 -m payload_catch.payload_model          # 自测
python3 -m payload_catch.rendezvous
bash tools/run_checks.sh --quick                # 一键自检（纯模块 + tools/test_*.py，秒级）
bash tools/run_checks.sh                        # 全量（+acados +offline --all +pytest）
bash tools/sitl_check.sh 70                     # SITL 端到端验收（需完整 SITL）
python3 tools/offline_run.py --all              # 13 个工况，全部 PASS
python3 tools/offline_run.py --scenario M4_high_kf --mc       # 蒙特卡洛
python3 tools/offline_run.py --scenario M2_line_v10 --compare # 开环 vs 闭环

# M6 垂直堆叠投放（A 正上方释放 + B 温和下潜 + 刚性漏斗）
python3 tools/stack_run.py                      # 单次
python3 tools/stack_run.py --sweep-dive         # 扫 B 下潜加速度
python3 tools/stack_run.py --sweep-gap          # 扫 gap
python3 tools/stack_run.py --mc 200             # 蒙特卡洛，200/200

# 真机末端：圆形托盘（内径30cm/围边5cm/泡棉e=0.15）接 6cm·100g 方块
python3 tools/stack_run.py --scenario M6_stack_tray --mc 300        # 99% (296/300)
python3 tools/stack_run.py --scenario M6_stack_tray_small --mc 300  # 25cm 盘：92%
python3 tools/stack_run.py --scenario M6_stack_tray_wind --mc 300   # +侧风3m/s：96%
python3 tools/tray_sizing.py                     # 托盘选型（内径/围边/e/gap）
python3 tools/tray_sizing.py --sweep-e           # e → 所需围边高度表
python3 tools/tray_sizing.py --measure-drop 1.0 0.22   # 落物试验反推 e
```

### 4.2 SITL（2 机 + Gazebo 载荷）
```bash
source ~/drone_payload_catch/env.sh
python3 tools/preflight_check.py --sitl                # 起飞前静态自检
python3 tools/preflight_check.py --live --gps           # 持续探测（EKF/磁/IMU/GPS/failsafe）
python3 tools/uplink_test.py --drone 0                  # offboard 上行链路验证
python3 tools/preflight_check.py --logs                 # 日志门（Ready / Gyro STALE / Arming denied）
bash ~/drone_payload_catch/run_m1_sitl.sh 50          # PD 控制器
CTRL=mpc bash ~/drone_payload_catch/run_m1_sitl.sh 50 # acados MPC
PREFLIGHT=1 bash run_m6_sitl.sh 70                    # 可选：READY 后 live 自检
# 可调几何：
A_HOVER="0.0,0.0,-3.0" B_STANDBY="0.2,0.0,-2.8" B_OFFSET="0.2,0.0,0.0" \
  B_POSE_ENU="0,0.2,0,0,0,0" CTRL=mpc bash run_m1_sitl.sh 45
```
结果看 `~/payload_catch_sitl/launch.log` 里的 `*** CAPTURED ***` 与 `B phase=...`。
⚠️ launch 向量参数**必须全 float**（`[0.2,0.0,-2.8]`，不能 `[0.2,0,-2.8]`，否则 launch 报类型不一致）。

### 4.2b M6 垂直堆叠投放（A 正上方释放 + B 温和下潜 + 刚性漏斗）
```bash
source ~/drone_payload_catch/env.sh
bash ~/drone_payload_catch/run_m6_sitl.sh 70
```
默认：A 悬停 4.5m、B 待命 3.5m、B 从水平 5m 外起飞对正。结果看
`~/payload_catch_m6/launch.log` 里的 `*** STACK CAPTURED ***` 与 `B phase=DONE`。
可调：`A_HOVER` / `B_STANDBY` / `B_POSE_ENU` / `B_OFFSET` / `RELEASE_OFFSET`。

### 4.3 构建本项目（改代码后）
```bash
cd ~/payload_catch_ws && source ~/drone_payload_catch/env.sh
colcon build --packages-select payload_catch
```
（`~/payload_catch_ws/src/payload_catch` 是软链到本仓库。改 yaml/setup 后需重新 build，
因为 launch 读 install 副本；但 **launch 参数可覆盖几何**，改参数不必 build。）

---

## 5. 里程碑进度与关键结果

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 骨架 + 载荷模型 + 会合规划 + 离线闭环 | ✅ |
| M2 | A 带速抛投（0.5/1.0/2.0 m/s + 斜向） | ✅ 离线全 PASS |
| M3 | 闭环重规划 | ✅ 释放误差 σ=0.2：开环 7/12→闭环 12/12；风+阻力失配：开环 0/10→闭环 10/10 |
| M3+ | acados 终端 MPC + 与 PD 对照 | ✅ 能接；**未超过** 解析前馈 PD |
| M4 | 载荷 KF 估计 + 延迟/丢包 + 蒙特卡洛 | ✅ KF 估计误差降 2–4 倍；但**成功率无提升**（真值基线同样 ~92% → 瓶颈不在估计） |
| B | 环境打通 | ✅ 用 PX4-1.16 解决（见 §2） |
| M1 | SITL 端到端（A 悬停释放 / B 会合） | ✅ |
| M5-1 | Gazebo 真实载荷 | ✅ |
| M5-2 | b_node 接入 acados MPC | ✅ |
| M5 step3 | 难度扫描找边界 | ✅ 见下 |
| M5-3 | 真实吸附机构（接住→带走） | ❌ 未做 |
| M6 | 垂直堆叠投放（A 正上方释放 + B 温和下潜 + 刚性漏斗） | ✅ 离线 200/200；**SITL 捕获成功** |
| M6-SITL | 5m 起飞→对正→释放→下潜→漏斗捕获→**两机分开落地** | ✅ `STACK CAPTURED`；两机落点相距~8.5m，均 `Landing detected`+`Disarmed by landing`；全程 min|A−B|≈0.9m（无碰撞） |
| M6 sweep | SITL 难度扫描（10 档，`report/m6_sitl_results.md`） | ✅ 9/10：重噪声+不滤波 ❌、其余 ✅。载荷闭环在 σ=0.20 时落点误差 3.7× 改善 |
| M6 MC | SITL 蒙特卡洛（N=5/档，`report/m6_sitl_mc.md`） | ✅ 标称 5/5；重噪声 滤波4/5 vs 不滤波1/5；释放误差0.20 跟踪5/5 vs 不跟踪1/5；σ=0.30 跟踪5/5 |
| 交接探索 | 静态/动态/机械臂/增速度公式推导（`report/handoff_exploration.md`） | ✅ 含 apex-catch ≥2g 负结果、反向抛投消速、臂 reach/吸收 |
| M6 干扰 | 侧风鲁棒：速度前馈 + 预测式对正（`report/robustness_wind.md`） | ✅ 侧风容忍 ~1→~3.5 m/s；噪声下降回 ~1.5 m/s（瓶颈=估计器） |
| M6 鲁棒 II | KF/增广风KF + 自适应下潜 + 漏斗/机械臂几何（`report/m6_robustness_opt.md`） | ✅ 末端能力主杠杆：臂reach0.10(eff_r.15→.25)侧风11→19/20，臂absorb2修下击暴流0→20/20；自适应修wz6 0→20/20；新增`BallisticDragKF`(估常值风)为最佳估计模式 |
| M6 末端能力 | 大漏斗 0.20→0.30m：新模型`x500_funnel_big` + `FUNNEL_MOUTH` 参数化（`report/end_effector_bigfunnel.md`） | ✅ 离线捕获余量 min 0.0076→0.1076m(×14)；SITL 标称与加噪声均 `STACK CAPTURED`(horiz 0.028/0.030m)、无 failsafe |
| 鲁棒几何+末端机构 | `geom_opt.py`/`funnel_model.py`（`report/robust_geometry_and_retention.md`） | ✅ 风下**免下潜规则 `gap≤v_retain²/(2g)`**；wind3 gap0.6/a_dive0 95% vs 标称78%；空心锥提水平/倾角容差，主动保持绕过 v_retain |
| 悬停优先控制 | `minimal_dive` 落到 `b_node`（`auto_min_dive`，默认 true）（`report/hover_first_control.md`） | ✅ 离线强横风 w=4: 4/30→27/30；SITL 标称 `a_dive=0.00(auto=True)` → `STACK CAPTURED horiz=0.061m`、无 failsafe |
| 抗阻力/风扰 | 二次阻力+阵风；A 端迎风预补偿 `a_wind_comp`（`report/drag_rejection.md`） | ✅ 估计器二阶(阵风良性/载荷低通)；**迎风预补偿(w8:0→40/40, 二次k.5:1→40/40)**，方向必须准、大小±30%宽容 |
| 空心导向锥杯 | `x500_funnel_cup` + `FUNNEL_TYPE=cup`（`report/hollow_funnel_cup.md`） | ✅ SITL 捕获；⚠️**负结果：敞口杯倾斜 25° 即滚出 vs 平盘摩擦 45°**；杯价值在导向/侧向兜接，保持/倾角需主动锁扣 |
| 主动保持锁扣 | 离线 `M6_stack_lock`(arm_absorb) + SITL `PAYLOAD_LOCK=1`（`report/active_retention.md`） | ✅ 离线：锁扣=去掉 v_retain，下击暴流 wz=8 仍 100/100；✅ **SITL：捕获时就地重生成载荷到 B 漏斗，锁定→随 B 携带→落地**（根因：`DetachableJoint` configure 即建关节，不可远处 spawn） |
| 真机末端·圆形托盘 | `M6_stack_tray/_small/_wind` 工况 + `tools/tray_sizing.py` 选型器（塑料围边+泡棉缓冲） | ✅ 离线：6cm/100g 方块，内径30cm/围边5cm/e=0.15 → **MC 99%**；25cm→92%；+侧风3m/s→96%；裸塑料 e≥0.4 → 0%。保持条件 `e²·gap ≤ h` |
| 真机末端·托盘 SITL | `FUNNEL_TYPE=tray`（`models/x500_tray` + `payload_100g`；`tools/gen_tray.py` 生成） | ✅ SITL **3/3**：`STACK CAPTURED horiz=0.015–0.049m rel_v≈2.7–2.8m/s`（v_retain=6.60）→ 捕获→载荷随托盘携带→双机分开落地；无 failsafe、min\|A−B\|≥1.08m |
| 托盘鲁棒·倾斜 | `tools/tray_tilt_test.sh`（倾斜落物台：托盘 vs 空心杯 vs 平盘） | ✅ 托盘保持到 **~40°**，平盘~20°不稳、空心杯20–30°反复；围边是关键（`report/m6_tray_robustness.md`） |
| 托盘鲁棒·侧风 | 离线风扫（`M6_stack_tray` + 风/阻力） | ✅ 100g 方块漂移小：风 0→6m/s 98→93%（无预测）；lead 对轻物反而略差 |
| 托盘·释放误差 | 离线 MC（对比大漏斗） | ⚠️ `eff_r=0.12` 是短板：σ=0.05/0.10/0.20 → 98/88/51%；大漏斗 eff_r=0.25 → 100/100/89% |
| 托盘·编队同速 | `models/payload_attached_100g` + 大托盘 `x500_tray_big`（`FUNNEL_TYPE=tray FORMATION_VEL≠0`） | ✅ 大托盘(40cm) 0.5/1.0 m/s：`STACK CAPTURED`+物理携带落地；⚠️ 小托盘(30cm) 运动中会被滑出（软件仍报 captured） |
| 托盘 SITL 扫描/MC | `tools/sweep_m6_tray_sitl.sh` / `mc_m6_tray_sitl.sh` | ✅ 扫描 7/8（仅重噪声不滤波❌）；MC：标称3/3、重噪声+LPF 3/3、释放误差0.20 **1/3** |
| 托盘·主动锁扣 | `FUNNEL_TYPE=tray PAYLOAD_LOCK=1` + `payload_lock_100g`（复用漏斗锁扣机制，托盘保留 funnel_link） | ✅ 小托盘(30cm) 定点/编队0.5/编队1.0 均**捕获→锁定→携带落地**；无 failsafe；运动交接与盘径解耦 |
| 托盘·泡棉 e 实测 | `tools/foam_drop_test.py`（H,h→e/TRAY_E/v_retain/最大gap）+ `tools/gz_foam_drop_test.sh`（虚拟落物台） | ✅ 虚拟落物台：模型泡棉 e≈0（比真机合格≤0.2 还死→仿真偏乐观）；真机应实测 e 后重算 TRAY_E |
| **真机接入层** | `relnav.py`/`relnav_node.py`（RTK 驱动→`/drone_a/state`）+ `contact_detect.py`（接触触发）+ `launch/catch_real_launch.py`/`run_m6_real.sh` + `report/real_hardware_bringup.md` | ✅ 纯逻辑自测通过；SITL 验证：`contact_detect:=true` → `检测到接触事件→捕获`；默认关闭无回归（托盘 SITL 仍 3/3） |
| **相对不确定度修正** | `uncertainty.py`（σ_rel=√(σ_A²+σ_B²−2ρσ_Aσ_B+(lσθ)²+σ_meas²)）+ `tools/rel_sigma.py` + b_node `sigma_model`/a_node `gate_use_relative` | ✅ 量化：绝对 σ（0.212）证书**不可行**；相对传感器 σ=0.03 可行、释放率 52%/条件捕获 0.999；SITL：证书闸修正后放行（`STACK CAPTURED horiz 0.045/0.056m`）；`report/rel_uncertainty.md` |
| **动力学/接触加强** | `dynamics.py`（倾角+推力约束）+ `impact.py`（冲击/可恢复性/带载余量）+ `tools/dynamics_contact.py` + b_node `miss_timeout_s`（接空安全中止） | ✅ 量化：`a_max=6`↔~31.5°倾角、水平权限随下潜衰减；100g 冲击可恢复、≥1kg 超权限；SITL 接空 → `MISS 安全悬停→降落`（`safe=OK`，无 failsafe）；`report/dynamics_contact.md` |
| **感知/接触/风 保真** | `perception.py`（相机模型）+ `impact.compliant_contact`（柔性接触）+ `tools/perception_study.py` + `tools/make_wind_world.py`（SITL 风场） | ✅ 相机模型替换真值替身（好相机近场更优、差相机大释放误差 14%）；柔性接触峰值力 1170→37N（30×）；SITL `WIND=6` 带风跑通；`report/perception_study.md` |
| **统计+基线** | `stats.py`（Wilson CI + McNemar）+ `tools/stats_report.py` | ✅ M6 大 N（每格 N=2000，CI）；感知(camera 100% vs 替身 98.7%, p<1e-3)、预测对正(lead=1 反而 79%<89%, p<1e-40)、闭环(0/30→28/30, p=3e-7)、释放误差敏感性、参数不确定性；`report/statistics.md` |
| **离线自检/回归/CI** | 抽 `payload_catch/safety_logic.py`（纯函数）；`tools/test_*`（C1 证书 / C5 CBF / 安全层 / **yaml 契约** / **无 ROS 守卫** / **全仓编译** / **起飞前自检逻辑**）；`tools/preflight_check.py`（起飞前自检：离线/`--sitl`/`--live`/`--gps`/`--logs`）+ `tools/live_probe.py`（持续探测 EKF/failsafe/磁/IMU/GPS/电池）+ `tools/uplink_test.py`（上行链路）+ `payload_catch/telemetry.py`/`tools/check_log_validity.py`（遥测有效性）+ `payload_catch/fcu_params.py`/`tools/preflight_params.py`/`tools/fcu_configure.py`（**飞控参数体检**：THR_MIN<THR_HOVER/失效保护/围栏/EKF 源）；`tools/smoke_acados.py`；`tools/sitl_check.sh`（SITL 验收）+ `tools/sitl_safety_matrix.py`（**故障注入矩阵 5/5**）；一键 `tools/run_checks.sh` + `Makefile` + pre-commit；`tests/`+`pytest.ini`；`.github/workflows/checks.yml` | ✅ **32/32 通过**（pytest 13）；顺带修好 `mpc_terminal` 自测三元组解包 bug + 硬化 `sim_core`/`stack_drop` 自测为带断言；CI 待首个 PR 验证 |
| 双机协调 | `rendezvous.solve_cooperative` + 协议分析（`report/coordination.md`） | ✅ 现状=单向/A被动；改进=握手/意图/时钟/安全；⚠️**负结果：A 小幅释放偏移与 t_r 冗余（δ*=0）**，协同价值在协议与 A 的速度/高度配合 |
| 协同释放握手 | `coord_mode=handshake`（`a_node`/`b_node` + `/drone_b/ready` + `/drone_a/release_cmd` + `/drone_a/intent` + 时钟同步 `/coord/ping|pong` + 释放门限）（`report/coordination_handshake.md`） | ✅ **SITL：B 报就绪→A 作释放权威（精确自身状态）→ack→B 下潜→`STACK CAPTURED`**；含往返**时钟同步**(offset≈0/rtt0.5ms 换算 t_rel) 与释放前一致性门限；默认 direct 保留 |
| 编队握手+意图 | M6-moving FORMATION 握手 + `/drone_a/intent` 升级为**预测落点**（`use_intent`/`WIND_EST`）（`report/coordination_handshake.md`） | ✅ SITL：`/formation/start`→FORMATION→**A 释放权威**→DIVE→`STACK CAPTURED horiz=0.040m`；意图落点(风漂移) SITL no-op 验证；**ack 改为原子事件**（不再依赖瞬时对齐） |
| 安全层 + PX4风 | keep-out=`min_ab_gap+kσ`（在线估 σ）+ 释放后 A 定向清场；`use_px4_wind`（订阅 `/fmu/out/wind`）（`report/coordination_handshake.md` §4d） | ✅ SITL：`SAFETY_FLOOR=0.2` 时 `WAIT_A clear=1.37≥gap_eff=1.2`、B 保持更低、`A: 开始定向清场`、`STACK CAPTURED horiz=0.010m`；PX4 风 hookup 不报错（x500 无空速，风估计可能恒 0，`wind_est` 作兼底） |
| 协同协议验证 | `tools/validate_coord.py`（`report/coordination_validation.md`） | ✅ 3 组配置（标称/噪声+安全/编队）**全部通过**：捕获、就绪→释放→ack 顺序、单次释放、min\|A−B\|≥1.0m、无 failsafe |
| 规划/协调+控制优化 | ZEM 终端导引(`zem_gain`) + 释放前落点余量闸（`report/planning_control_opt.md`） | ✅ 离线：ZEM 临界风 w4 28→36/40、w5 13→22/40（`zem≈1`）；SITL `COORD=handshake ZEM=0.8` → `STACK CAPTURED horiz=0.055m`、无 failsafe |
| 安全监督(kill) | `px4_iface` 安全监督：`/safety/kill_a|b` 外部 kill + 异常(姿态/越界/状态超时)持续自动 kill（`MAV_CMD_DO_FLIGHTTERMINATION`）（`report/safety_supervisor.md`） | ✅ SITL：发 `/safety/kill_b` → B `Flight termination active`（动力切），**A 不受影响**（计数 0） |
| **companion 安全网** | 失联看门狗（`safety_logic.peer_loss_action` + b_node：丢 A 状态→就地冻结→AUTO.LAND）+ 硬碰撞地板（`keepout.hard_floor` + b_node→`external_safety_reasons`→HOLD）（`report/companion_safety.md`） | ✅ 纯逻辑单测 + **SITL 诱发**：kill A → `失联 1.0s 冻结`/`4.0s LAND`；`collide_emerg:=1.5` → `SAFETY HOLD: collision_floor(d=1.43)`；标称零误触发 |
| 优化第二轮 | 安全分级状态机(OK/HOLD/PULLBACK/LAND/KILL) + 越界回拉；DIVE 加速度前馈(PX4 `trajectory_setpoint.acceleration`)；B 在线 σ 共享给 A 释放闸；释放提交窗口+lead 窗口取消；3D 反应式 keep-out；在线自适应下潜（`report/opt_round2.md`） | ✅ 单元+多轮 SITL：标称 `STACK CAPTURED horiz 0.008–0.080m`、双机落地、无 failsafe；越界 `PULLBACK`、丢状态 `HOLD`（自愈） |
| 优化第三轮 | 安全硬化：安全指令绕过 `sp_rate_limit`；围栏改 `_fence_velocity` 软限幅（保留切向、消除与任务互顶）；`safe=` 进周期日志（`report/opt_round3.md`） | ✅ 单元 5 项 + SITL（围栏 3m 稳定、标称 `horiz=0.018m` 无 failsafe） |
| 优化第四轮 | 传感器/估计器约束（零新硬件）：用 `vehicle_local_position` 未用字段 — `eph/epv` 作 σ、健康/一致性看门狗(`dead_reckoning`/valid/`reset_counter`)、估计器限值(`vxy_max/vz_max/hagl_min`)（`report/opt_round4.md`） | ✅ 单元 5 项 + SITL（启动期 `estimator_reset`→1s HOLD 自愈、标称 `horiz=0.034m` 无 failsafe） |
| 优化第五轮 | 姿态/角速率约束（IMU→安全滤波）：倾角/角速率衰减水平指令 + 水平指令加速度限额；对标 `v_p≈4.9` 姿态 failsafe（`report/opt_round5.md`） | ✅ 单元 5 项 + SITL（标称 `att=1.00` 全程不误触发、`horiz=0.032m` 无 failsafe） |
| M6-moving 加速度 | 编队控制优化（死推算参考+稳释放门限）+ 空心杯保持 + **及时释放/超时保护** + 多重复验证（`report/m6_moving_speed.md`） | ✅ **支持速度 0.5/1.0 m/s 各 2/2 完美**（捕获+保持+双机落地、无 failsafe）；✅ **及时释放**（编队 ~3.5s 即捕获，之前 10–20s）+ **超时中止**（不投、A 保留载荷、双机安全落地）；**2.0 m/s 保留、不再优化** |
| M6-moving | 编队同速投放（同一投影点→同向同速巡航→运动中释放，物块继承 A 速度） | ✅ 无窗口 SITL 3/3（`STACK CAPTURED horiz 0.108–0.128m`，物块随漏斗落地）；GUI 偶发平台 failsafe |
| 优化 2–5 | 分级安全状态机/越界软限幅/加速度前馈/σ共享/释放提交取消/3D keepout/自适应下潜；传感器约束(EKF σ/看门狗/限值)；姿态安全滤波 | ✅ 单元+多轮 SITL（`report/opt_round2..5.md`）；标称 `horiz 0.008–0.08m` 无 failsafe |
| 研究·协调 | 路线图 C1–C5（`report/research_roadmap.md`）；阶段 0 协同基准 `tools/bench_coord.py` | ✅ 基准；✅ **理论 C1–C5 + T1–T3**（证书/最优性/协议/信息/机动/CBF/联合证书/延迟 CBF）；✅ 证书闸+CBF+C2 冗余接入 SITL；✅ **实验主表**（逐项消融，stress 下时延 3.3→0.72→0.42s，`coordination_experiments.md`）；⏳ 真机 & 写作 |
| 真机 | — | ❌ 未做 |

**SITL 难度扫描结果**（`report/m5_sitl_results.md`，MPC 控制器）：
- 偏移 0.2/0.5/0.8 m（A 3.0 m，v_p≈2 m/s）：**全部捕获**，可重复。
- A 4.0 m（v_p≈4.9 m/s）：**全部失败**，B `Attitude failure (roll)`/`Compass`+`Battery` → **failsafe**。
- 结论：横向偏移不是瓶颈；**真瓶颈是载荷下落速度（B 的末端俯冲）**。

---

## 6. 关键设计决策与负结果（避免重走弯路）

1. **A 运动第一版用恒速直线**（`a_vel=0` 即悬停）；载荷 = 无阻力抛体，可选阻力/风。
2. **协调搜索**：在 `(t_r, τ_c)` 网格上最小化
   `J = w_time·t_r + w_accel·(峰值a/a_max) + w_vel·|Δv|² (+ w_overshoot·过冲)`。
3. **B 会合轨迹**：min-energy 三次多项式（解析），终端速度精确匹配；不可行时退**软终端速度**。
4. **`solve_inflight`**：载荷离手后的重规划入口（闭环，抗释放误差/模型失配）。
5. **负结果 A：终端 MPC 没有赢过 PD。** PD 带解析 min-energy 前馈，在简单双积分器里已接近最优。
6. **负结果 B：过冲不是缺陷。** min-energy 参考在"时间充裕、终端速度大"时会先爬升再俯冲；
   强行用 `w_overshoot`/`staged` 参考去掉过冲 → 用满 `a_max`、**丢失反馈余量** → σ=0.15 成功率 8/10→6/10。
   **默认保留 `cubic` 参考**。真正的改进方向是从**会合几何**（给 B 下滑跑道）入手。
7. **负结果 C：更好的估计器不提升成功率。** KF 估计误差降 2–4 倍，但用真值的 `none` 基线同样只有 ~92%
   ⇒ 瓶颈是释放误差+动力学，不是估计精度。
8. **`release_mode`/`ctrl` 的默认**：`reference_mode=cubic`、`controller='pd'`（launch 默认；SITL 用 `CTRL=mpc`）。
9. **M6 用专门的垂直投放规划器，不复用 3D 会合搜索。** 3D `RendezvousPlanner` 求最小代价时会让 B
   **爬到 A 正下方 ~0.1m 处**（只掉 10cm 就接住），或强行末端速度匹配（需峰值加速度 15–600 m/s²，不可行）——
   都不是“B 在下面等、载荷掉下来”。故新增 `stack_drop.py`（解析、纯 Python）。
10. **M6 关键物理：纯垂直下落的接触相对速度有下界** `v_rel = sqrt(2·(g − a_dive)·gap)`。
    B 只能往下压（a_B<g），提前下潜/变速下潜都不能降低它。gap=1m：悬停硬接 4.43 m/s，
    a_dive=3（温和）→3.69、a_dive=6（全速）→2.76。下潜越猛 v_rel 越小但 B 冲得越低、刹车余量越少（a=6 时
    刹停后仅 0.35m）。**a_dive=3 是冲击/余量的折中**。
11. **M6 判据不用固定 `v_c=1.5`，改用刚性漏斗物理保持速度** `v_retain = sqrt(2·g·depth)/e`
    （由反弹高度 e²v²/(2g) ≤ depth 导出），位置判据 = 落到漏斗口平面时水平偏差 < mouth_radius − object_radius。
    固定 1.5 会误杀本来能接住的工况（1m 落差的物理下界就有 2.76）。

---

## 7. 已知坑 / 陷阱（血泪）

1. **`pkill -f <pattern>` 会把执行命令的 shell 自己杀掉**，如果命令串里含同样 pattern
   （如 `pkill -9 -f 'px4'` 在含 `px4` 路径的命令里）。务必把 pkill 写进**独立脚本文件**再执行。
   ⚠️ **本会话又踩两次**：即使 pkill 在独立脚本里，**外层命令行**若出现同一模式的字面串
   （如外层命令里写了 `catch_stack_launch.py` 而清理脚本 `pkill -f 'catch_stack_launch'`），
   仍会自杀。⇒ 把“触发 pkill”的调用与含该模式串的命令**分两次调用**。
2. **launch 参数向量必须全 float**（见 §4.2）。
3. **`setup.cfg` 的 `install_scripts` 不要指到 `/usr/local/bin`**（需 root，构建失败）；用 ament 标准 `$base/lib/payload_catch`。
4. **acados json 路径**：新版把 json 写到 `c_generated_code/`，指纹缓存要按这个路径判断，否则永远 miss。
5. **改了 OCP 结构/权重** → 清 `~/.cache/payload_catch/acados_terminal_mpc/`（指纹其实会自动失效，但保险起见）。
6. **drone1（B）固有健康告警**：`Compass 1` + `Battery unhealthy`，参考项目带着它也能飞；但**额外负载**
   （全量订阅 `/world/default/pose/info` + 现场编 acados）会顶出 gz 时钟抖动 → EKF 姿态失效 → failsafe。
   **对策（已实施）**：载荷只订阅自身 `/payload/odom`；MPC 用指纹缓存 + `tools/prebuild_mpc.py` 预热；
   PX4 启动间隔 12s、等双机 Ready 后额外等 8s 让 EKF 稳。
7. **载荷捕获后会与 B 碰撞被弹开**（因为还没做真实吸附）——软件判据在碰撞前已触发，属预期。
8. **Gazebo `create` 服务首次调用可能 ~5s 超时**：所以 `payload_node` **启动时**就把载荷生成在远处停车位
   (ENU 100,100)，释放时用 `set_pose` 瞬移到释放点（避免释放时刻的服务延迟）。
9. **b_node 早退陷阱**：`control()` 不能用 `p_pay is None` 早退，否则 HOLD 阶段永远执行不到规划。
10. **参考轨迹执行完后不能继续用末点速度前馈**（会让 B 一直俯冲砸地）；结束后应在会合点悬停。
11. **`mpc_terminal.solve` 返回三元组 `(u0, status, v_next)`，`sim_core` 曾按二元组解包** →
    `--all` 跑到 MPC 工况会 `ValueError: too many values to unpack`。已修为 `_out[0], _out[1]`。
    （`v_next` 供 PX4 速度接口当前馈设定点，离线仿真忽略。）
    ⚠️ **同一 bug 也藏在 `mpc_terminal.__main__` 自测里**（长期未暴露，因为没人聚合跑退出码）——
    已修，并被 `tools/run_checks.sh` 覆盖。教训：自测带断言 + 聚合跑退出码，否则等于没有。
12. **PX4 `pos_world.z` 比模型绝对高度低 0.24m**（x500 `base_link` 在模型 z=0.24）。
    载荷 odom 是绝对高度，所以漏斗口判据必须带 `px4_z_bias=0.24`，否则永远不触发捕获。
13. **给单台载机加自定义模型不改 PX4 树**：先 `gz service create` 成 `x500_funnel_1`，
    再以 `PX4_GZ_MODEL_NAME=x500_funnel_1` 启动（占 `px4-rc.gzsim` 的 `elif` attach 分支，不 spawn）。
    模型用 `<include merge='true'><uri>model://x500</uri></include>` 复用标准 x500（含 IMU/mag/螺旋桨插件）。
14. **gz `<cone>` 默认尖朝上**（载荷会滑落）；要口朝上（漏斗形）必须 `roll=π`。
    且**实心圆锥宽口朝上 = 平顶盘**，并不是空心漏斗（想要真漏斗得用内壁网格）。
15. **M6 载荷释放点要在 A 下方 `offset=0.15m`**，否则在 A 机体/桨内生成会被瞬间弹飞。
16. **M6 起飞阶段必须“先垂直爬升 → 等 A 到位 → 再横移”**。若 B 从 5m 外直接斜插到 A 正下方，
    斜插路径会穿过 A 的垂直爬升通道，两机在 (0,0) 附近高度交叉时**相撞**（实测 B 被撞出 0.8 m/s 横向速度）。
    现相位：`CLIMB`(垂直爬升) → `WAIT_A`(等 A 到悬停高度且 clear≥`min_ab_gap`) → `TRANSLATE`(定高横移) → `ALIGN`。
    另：`DONE` 阶段**不能每拍把悬停目标重锚到当前位置**（带载会缓慢漂移靠近 A），改成捕获瞬间锁定悬停点。
    `b_node` 全程跟踪 `min_relA`（最小 A-B 间距）作碰撞监测，安全层保证 B 高度 ≤ A−`min_ab_gap`。
17. **M6 相对定位估计必须 EMA 低通**（`est_lpf_alpha≈0.30`）：重噪声下若直接用带噪 `rel_xy`
    做对正门限，1s 内几乎进不了 0.12 阈值 → B 卡在 ALIGN 永不释放（扫描 T3n 实测）。滤波后噪声 ↓~3×。
    另：`min_relA` 由**估计值**算出，重噪声下会虚低（T3n 显示 0.481，实际 ~1m），别当碰撞。
18. **M6 参数已全部参数化到 launch**（可 `ros2 launch payload_catch catch_stack_launch.py rel_pos_sigma:=0.1` 覆盖）；
    扫描脚本 `tools/sweep_m6_sitl.sh` 每档重启 gz+PX4；首轮 `pkill` 竞态会令“双机未 READY”，脚本已自动重试。
19. **M6-moving（编队同速投放，`FORMATION_VEL`≠0）的两个专用坑**：
    - 挂载型载荷 `models/payload_attached` 的 `DetachableJoint` **在模型 configure 时就建关节**；
      载荷必须在 **A 正下方生成**（`_spawn_attached`），且**不可再发 attach**，否则叠加第二个
      固定关节 → 过约束 → 分离瞬间物块被甩飞（实测 ~6.8 m/s）。只需在释放时发 detach。
    - B 的 `FORMATION` 高度控制符号：`vz = -kp*(alt_target - (-pos_z))`（与 `hover_velocity` 一致）。
      写反会变成正反馈 → B 一路爬升撞上 A 下方挂的载荷。仿写控制律时务必对 NED 符号。
    - 挂载后 A 下方物块与 B 漏斗顶部只有 ~0.5m 间隙，B 必须严格 ≤ A−`min_ab_gap`。
20. **M6-moving “接不住”的四个真因（已修，记下来）**：
    1) **捕获误触发**：物块还在 A 上/刚分离就满足漏斗口窗口 → B 切 DONE 停止跟踪，物块真下落后漏接。
       修：DIVE “重锚”——未检测到物块下落（`vz>dive_anchor_vz`）就**原地悬停等**，不早下潜；
       捕获窗口改为漏斗口上下 `±catch_z_tol`。
    2) **DIVE 等待期用 `standby` 悬停**：编队模式下 B 已在 x≈几米处，控制器把它拉回**原点**，
       导致与物块擦肩。修：改为**在原地（当前 pos）悬停**。
    3) **物块与 A 起落架干涉**：A 起落架下伸 ~0.23m，`RELEASE_Z=0.30` 时物块与腿只剩 ~0.04m，
       挂载期间接触力积累 → 分离瞬间被弹飞。修：`RELEASE_Z=0.45` + A/B 间距拉到 1.7m（A 5.0/B 3.3）。
    4) **横向 P 增益过大**：相对定位 EMA/延迟下 `stack_kp_xy=2.5` 产生速度振荡（0.02↔0.83），
       对正/跟踪都不稳。修：降到 **1.2**，且编队/DIVE **跟踪 A**（无释放误差）而非带噪的物块估计。
    另：独立物理试验确认 gz 漏斗（cone roll=π）确实是**平顶盘**、能接住垂直下落的物块。

---

## 8. 当前边界与未做

- **支持工况**：M6 垂直堆叠（离线 200/200；SITL 捕获+携带+双机落地）；
  **M6-moving 编队同速 0.5/1.0 m/s（各 2/2 完美）**；侧风容忍（纯风 ~3.5 m/s、加噪 ~1.5 m/s）；
  大漏斗 `eff_r=0.25m`；主动保持锁扣（下击暴流 8 m/s 离线 100/100）；
  **真机圆形托盘**（内径30cm/围边5cm/泡棉 e≤0.15，接 6cm·100g 方块 MC≈99%；25cm→92%）。
- **已知边界**：常规下 `v_p≈4.9 m/s` 触发 B failsafe（飞控/传感器，非算法）；
  **2.0 m/s 编队保留、不优化**；敞口杯倾斜 >25° 会滚出（不如高摩擦平盘 45°）。
- **未做**：真空心漏斗 / 夹爪 / 磁吸；真机；户外/RTK；真实相对导航（现为真值+噪声）。
- **研究线索（实证）**：协同基准显示释放闸的 σ 被 **A 自身 `eph`（≈0.15m）** 主导（`σ_used=0.150`）——
  “绝对位置 σ”当“相对交接 σ”过度保守（A/B 误差部分抵消）。这是 **C1：给“相对 σ + `P(capture)≥1−ε` 证书”** 的直接动机。

---

## 9. 下一步候选（按价值）

1. **真空心导向漏斗 + 真保持机构**：现为 primitive 薄壁杯（数值偏脆）+ 刚性 `DetachableJoint` 锁扣；
   下一步 mesh 内壁 + 内唇 + 夹爪/磁吸。
2. **推高速度边界**：修 B 的 EKF/磁罗盘鲁棒性（现 ~4.9 m/s failsafe），或末端俯冲更平滑。
3. **带载操控/落点精度**：锁扣后 B 的质量/惯量突变 → 自适应控制。
4. **安全层补全**：越界回拉、接不住 abort/go-around、反应式避碰、低电/超时。
5. **真机化 / 真实相对导航（UWB/视觉）**；2.0 m/s 编队暂缓。

---

## 10. 提交历史（git log，自上而下）

```
bfe0ac0 feat(research): 统计 + 基线（Wilson CI + 配对检验 + 参数不确定性）
4a8a1cf feat(research): 感知/接触/风 保真（补 sim-to-real 缺口）
380c7f5 docs(skill): 补充在 pi 中使用该 skill 的接入说明
d4e2dca feat(skill): 提取 Agent Skill aerial-payload-handover（SKILL.md + references + scripts）
7293881 feat(research): 动力学/接触纳入保证（控制层/安全层加强）+ 接空安全中止
6fafc62 feat(research): 相对不确定度模型——修'绝对 σ 当相对 σ'的过度保守（C1 证书）
3c6b020 feat(real): 真机接入层——相对定位(RTK)驱动 + 接触检测 + 真机 launch/文档
e5ad53d feat(M6): 托盘主动锁扣（运动交接去盘径化）+ 泡棉 e 实测工具
a06ef60 feat(M6): 托盘 SITL 难度扫描/MC + 倾斜/侧风鲁棒 + 编队同速接入
62b248c chore: 移除误提交的 Word 临时锁文件 + gitignore ~$*.docx
4c769f4 feat(M6-SITL): 真机末端圆形托盘接入 SITL（FUNNEL_TYPE=tray）
115823f feat(report): Word 仿真报告生成器 + 报告
        （report/make_docx_report.py；report/drone_payload_catch_sim_report.docx）
bff54a5 feat(research): 加 MODE=full 预设 + T3 延迟鲁棒 CBF 接入节点
        （run_m6_sitl.sh MODE=full；b_node.keepout_delay_s；SITL horiz=0.043m）
3a2619e docs: 同步研究进度到说明/报告文档（README/MEMORY/paper_outline）
fdf5476 feat(research): 逐项消融实验主表（--grid ablation [--stress]）+ 实验报告
        （应力下释放时延 3.3→0.72→0.42s；report/coordination_experiments.md）
724834e feat(research): 理论补强 T1/T2/T3——最优性 + 联合证书 + 延迟 CBF
        （report/coordination_theory.md；coord_optimal.py/coord_cbf.py）
83641e0 feat(research): 整合 C5 CBF + C2 冗余触发入节点 + SITL 全套验证
        （keepout_mode=cbf / /payload/released 冗余触发；payload_catch/keepout.py）
67e8749 feat(research): C5 handover-CBF 交接安全证书（速度级防碰 + C1 交接门）
        （可行域内 CBF 100%；report/coordination_cbf.md；理论五件套 C1–C5 齐备）
ba64a61 feat(research): C4 A 速度/高度 × B 机动联合优化
        （速度—余量权衡/联合几何恢复余量/顺风耦合；report/coordination_maneuver.md）
9a23e14 feat(research): C2 交接协议时序误差界与鲁棒性
        （release_lead=d_max/(2(1−catch))；丢 cmd→未下潜；report/coordination_protocol.md）
801529f feat(research): C3 state vs intent 延迟容忍（离线概率模型）
        （intent 延迟不变/保留证书；state 随 v_A·d 崩；report/coordination_information.md）
3167352 feat(research): 协同基准增加“证书闸 vs 启发式”消融（--grid gate）
        （噪声下释放时延 ~2.4s→~0.42s，≈5×；report/coordination_probability.md §8）
73626d7 feat(research): C1 证书闸接入 a_node/b_node（相对 σ）+ SITL 验证
40f6d62 feat(research): C1 v2——精确 Rice 证书 + EMA 标定
3b826a0 feat(research): C1 释放决策的概率模型与捕获概率证书（W2）
        （tools/coord_prob.py + report/coordination_probability.md）
133741c docs: 新增论文骨架（协调方向）——锁定论点/贡献 C1-C5/形式化/实验/相关工作/真机/时间线
fe2d73c docs: 将协调研究成果与发现并入说明/报告文档（README/MEMORY/research_roadmap）
3cfcdb6 feat(bench): 协同交接基准实验台（研究阶段 0）
        （tools/bench_coord.py + report/coordination_benchmark.md；grid×CI）
371b736 docs: 新增研究路线图（协调方向）——C1–C5 贡献/现状差距/理论工具/实验方案/代码映射/MPU
e574d32 feat: 姿态/角速率约束第五轮——IMU 安全滤波
        （_attitude_govern 倾角/角速率衰减水平指令 + 加速度限额；report/opt_round5.md）
6ba14bf feat: 传感器/估计器约束第四轮——用 PX4 已有字段（零新硬件）
        （eph/epv 作 σ / 健康看门狗 / 估计器限值；report/opt_round4.md）
b9bafaf feat: 安全硬化第三轮——软围栏/安全指令绕过限幅/安全状态进日志
        （_fence_velocity 软限幅 / sp_rate_limit 仅 OK 态 / safe= 周期日志；report/opt_round3.md）
e13483d feat: 优化第二轮——分级安全/加速度前馈/σ共享/释放提交取消/3D keepout/自适应下潜
        （px4_iface 分级安全状态机 OK→HOLD→PULLBACK→LAND→KILL + 越界回拉 / DIVE 加速度前馈 /
         B 在线 σ 共享给 A 释放闸 / 释放提交窗口 + lead 窗口取消 / 3D 反应式 keepout /
         在线自适应下潜；新增 report/opt_round2.md）
f65f089 docs: 同步工程现状（README 路线图/能力速览、MEMORY 现状/边界/下一步/提交史、SIM_COMMANDS）
d82b839 feat: 鲁棒性/末端/协同/安全 系列优化 + 完整文档与验证
        （探索公式 / 抗风抗阻 / 增广风KF / 大漏斗 / 空心杯 / 主动保持锁扣 /
         协同握手+意图+时钟 / 安全监督kill / ZEM / 余量闸 / M6-moving加减速与及时释放超时中止；
         已在 GitHub: lowtobeking/drone_payload_catch）
fd22613 test(M6): SITL 蒙特卡洛(N=5/档) — 滤波与载荷闭环的必要性(1/5 vs 4-5/5) + 报告
0c86405 docs(M6): SITL 难度扫描报告 + 相对定位/滤波/闭环发现记录
8743617 feat(M6): 相对定位细化(抖动/丢包/慢变偏置/种子) + 载荷闭环跟踪 + EMA滤波 + 参数化launch + SITL扫描脚本
f1628b6 feat(M6): 两机落地点分开（各飞 land_xy 再降落，相距~8.5m）
8b44b1e feat(M6): 捕获后保持6s → A/B 各自 AUTO_LAND 落地收尾（PX4 Landing detected+Disarmed）
fbfb181 docs: 记录 M6 起飞避碰策略 + min_relA 监测 + GUI 脚本
4d62d33 fix(M6-SITL): 消除起飞期 A/B 碰撞 — B 先垂直爬升→等A到位→再横移；min_ab_gap 安全层 + 捕获后锁定悬停点防漂移
f74f495 docs(MEMORY): 记录 M6-SITL 提交哈希
72134b1 M6-SITL: 5m起飞→对正→A正上方释放→B温和下潜→刚性漏斗捕获(STACK CAPTURED) + 自定义x500_funnel模型/相对定位/状态机
39adbd2 docs(MEMORY): 记录 M6 提交哈希
9e36729 M6: 垂直堆叠投放离线层(解析规划+刚性漏斗判据+200/200) + 修 sim_core MPC 解包
bf467d1 docs: 新增 MEMORY.md 项目记忆(给下一个 AI 直接接续) + README 指针
16fcdf8 M5 step3: SITL 难度扫描 + 结果记录
4fb558d M5 step1+2: 降负载(载荷 odom) + acados MPC 指纹缓存 -> B 无 failsafe, MPC 捕获成功
f382a0a M5 step2: b_node 接入 acados 终端 MPC (controller:=mpc), 预测速度作 setpoint
118cce3 M5 step1: 温和场景 SITL 干净捕获(Gazebo 载荷)
```

> 注：`b78bf5c`/`60b0847` 记录的是"在错误的 PX4 main 树上排查"，**结论已被 §2 取代**——
> 直接看 §2，不必重读那两个 commit 的细节。

2026-10-15 释放延迟验收记录：report/release_delay_20261015/validation.json；工程quick 34/34、释放单元28项、安全运行23项、双机启动参数及本机ROS正常释放/动作前急停/动作中急停全部通过，git diff --check通过。测试仅Mock；未连接或部署Jetson。初次quick的测试管道竞态已修正，历史日志保留quick_initial_test_race.log。

2026-10-15 直出验证：工程quick34/34、GUI模拟直接1700µs、ROS正常/前后急停、独立2秒上限与0.25秒失联保护均通过；证据report/direct_release_20261015。旧缓变1.5秒未被作为有效直出标定，未动作硬件/部署。

2026-10-15 时序对齐最终验证：工程quick34/34、释放测试32项、安全运行24项、ROS正常/动作前中急停与Fast DDS双进程通信全部通过，git diff --check通过。证据report/timing_alignment_20261015/validation.json；未部署或驱动硬件。

2026-10-15 用户授权A机室内无桨部署已完成：nvidia@192.168.155.186，current=/home/nvidia/payload_catch_deploy/current，release=e27cb2caa69a0ff8fc9cf4d43adbcc3a40419dc07c803536187a1a3608a9d9cb、ROLE=a。ARM构建/32释放+24安全+10起飞测试/ROS隔离验收通过，控制输出均0。真实PWM disabled，当前period20ms/duty0/enable0，boot哈希未变。未启动任务/服务、未改旧ros2_ws。首次部署回退unselect_a.py只撤current链接保留版本。详见report/a_deployment_20261015/A机部署说明.md及日志。


## A 机独立起降验收（2026-10-19）

用户要求先验证 A、不依赖 B。新增独立 a_solo_validation_launch.py / deploy/run_a_solo.sh，默认 check/execute=false，起飞需当次 A-only 实时参数+遥测检查和显式 start；强制电量≥40%、定位、RC、飞控自检、身份/电池一致性及控制互斥。任务为低速垂直起飞1m、稳定悬停10秒、降落并确认着陆上锁；无负载释放/PWM/B接口，正式双机门禁不变。独立监督处理任务失联与故障，人工接管不抢回，stop降落/kill切动力分开。26逻辑+8隔离ROS场景及原双机10/安全24/定时释放10回归通过，Humble构建通过。本次仅Ubuntu22.04开发，未部署Jetson、未真实解锁或动作。上次实机无RC/水平定位且电池节数不一致仍是阻塞，未新读确认。操作与限制见docs/A机单机起降验收.md；证据report/a_solo_validation_20261019/validation.json。


## A 机单机 Gazebo 功能验证（2026-10-19）

本机 Ubuntu22.04 / PX4 1.16 SITL / Gazebo GUI 连续两次完成自检、Offboard、解锁、1m起飞、稳定悬停10s、正常降落并上锁；Gazebo最高z分别0.997m和1.009m。8个隔离ROS场景、26逻辑、10起飞门禁、7参数协议及飞控参数测试通过。修正转发GCS心跳、_HASH_CHECK元数据和BAT_*共享阈值兼容；仿真时钟仅隔离测试注入，真实默认不变。早期电池/磁力计遥测中断曾触发保护，根因尚未最终定位，不能据两次通过宣称长期稳定。模拟RC+外部定位，不验收真实A/B/PWM；本次未SSH/部署/真实动作。证据report/a_solo_gazebo_20261019/A机单机功能检查.md。临时编辑脚本已归档diagnostic_helpers，任务进程已退出。


## A 机单机版实机部署（2026-10-19，仅部署不运行）

用户最新指示：先部署，不用跑。已在 A 机 nvidia@192.168.155.186 完成 ARM64 构建，current 已切换至 b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37；previous 保留旧版 13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a。入口 /home/nvidia/payload_catch_deploy/current/src/deploy/run_a_solo.sh；配置 /home/nvidia/payload_catch_deploy/a_solo.yaml 保留 site_checked=false/payload_removed=false。此次未运行测试、ROS任务、飞行或PWM，未设置开机自启。文件/进程复核确认无任务和测试进程残留，服务、PWM、boot配置和host/runtime.yaml不变。单元测试此前通过；机载隔离ROS验收因新建report目录归属导致PermissionError，尚未通过。此前中断的目录修复未执行到源码，部署的是已编译的b4453dd版本。保留起飞门禁，不代表具备真实起飞条件。部署报告及精确回退命令见 report/a_solo_deployment_20261019/A机仅部署记录.md。


## A 机动捕与 SSH（2026-10-21，当前）

A 仍为 192.168.1.4，序列号1612425621719；用户将A刚体改名planeA。使用电脑临时辅助源地址192.168.1.3成功SSH，在A上原生VRPN接收192.168.1.111:3883，默认/显式绑定Wi-Fi两次各10秒均1199帧、约120Hz、无无效值/非递增时间戳；未转发飞控或动作。普通SSH失败已确认：电脑现IP192.168.1.2，A的有线192.168.1.101/24优先于Wi-Fi /22，给.2回包走错有线；我此前仅为旧电脑.3加/32路由的方案不完善。尝试新增.2路由后nmcli reapply未即时生效，已验证失败自动回退至原.3/.111两条路由。后续管理员UAC被取消，未继续修改。电脑临时.3已删、DHCP仍.2、共存选项恢复disabled，本任务进程已退出，不保持SSH；普通连接长期修复未完成。证据及细节见report/mocap_connection_20261019/A机动捕实测_20261021.md。


## A 机保持 SSH（2026-10-21，覆盖前条未修复状态）

用户要求现在保持连接，再次授权后已成功恢复：Wi-Fi profile 5bf25dec-358c-4a85-9eed-ee5b19a254ca 新增192.168.1.2/32 metric10；nmcli reapply 后等待约0.25秒，实际路由已走wlP1p1s0，不需手工ip route。由电脑正常源地址.2验证A序列号成功后commit。原.3/.111路由与其他网卡保留；网段重叠整体问题尚未解决，电脑未来换IP仍需处理。临时电脑.3已移除，原DHCP .2保留，管理员恢复PID26572已退出。保留用户明确要求的一个SSH交互会话：工具session_id=18922，ServerAliveInterval=15/CountMax=3，目标nvidia@192.168.1.4，HostKeyAlias=192.168.155.186；关闭须向该会话发送exit，不要误清理用户要求的连接。无飞行/PWM/ROS任务运行。正常SSH下再次在A原生采集planeA，10秒1198帧/120.0177Hz/invalid0/非递增0，forwarded_to_px4=false。记录report/mocap_connection_20261019/active_ssh_session_20261021.json，实时回包问题本次已修复，重启持久性未实测。


## A 机起飞条件实测（2026-10-21，最新）

按用户要求先重新SSH成功，序列号正确，boot846ff451-5740-4778-ba6e-71eaf9072602。在已部署环境只订阅8秒：77%/15.796875V/电池warning0，独立RC有效，自检标志true、已着陆未解锁、姿态有效；但xy/水平速度/可控航向无效，dead_reckoning=true，EKF全部EV融合false，无外部里程计输入发布者。电池仍cell_count1 vs expected4。配置仍outdoor_gps且无GPS，site_checked/payload_removed=false。GCS丢失/relay目标.123，power_input_valid=false需核查。未写参数、未发控制、未启动任务；完整安全参数清单本次未读，不能宣称已全部通过。SSH后期超时，电脑网卡已被改为手动192.168.1.124/24（非本轮修改），A现有/32只覆盖.2/.3/.111，未改用户手动配置；当前不宣称保活连接。报告report/mocap_connection_20261019/A机当前起飞条件_20261021.md。后续优先恢复网络、动捕桥接/EKF、修正电池来源后再做无桨接管/失联验收和A空载首飞。

2026-10-21 QGC连通后再次只读实测：电脑Wi-Fi恢复DHCP .2/22，有线.123/24，SSH正常；QGC gcs_connection_lost=false，relay双向计数增长，未改网络/转发。A原始planeA约120Hz/1199帧但未送入PX4；8秒遥测：69%/15.46V，电池cell_count1与预期4不符；xy/vxy/航向无效、dead_reckoning=true、EV融合全false，无外部里程计发布者；RC有效、着陆未解锁，配置仍outdoor_gps，现场/空载确认未登记。结论不满足工程自主起飞；未写参数或运行飞行/PWM。完整安全参数、RC接管/急停/失联仍待验收。证据report/mocap_connection_20261019/A机QGC连通后起飞条件_20261021.md。旧SSH会话74182已断线，按用户要求保留新的ssh -N会话28664（15秒保活）。

2026-10-21 动捕坐标转换预览：仅Ubuntu22.04工程新增mocap_transform/mocap_preview_node/config/mocap_a.preview.yaml及ROS入口。候选world->local=[Y,X,-Z]，机体->marker=[-Y,-X,-Z]，完整双侧姿态变换；候选安装、杠杆臂和服务器时序均未标定。10项逻辑测试、隔离ROS预览6接受/4拒绝/失效检测/fmu发布者0通过，实测位姿离线换算与手动左转相符。未部署A、未改参数、未向飞控发布，不是已完成PX4桥接。详见docs/A机动捕转换预览.md。测试进程正常退出。

2026-10-21 A机实时动捕转换预览完成：A=192.168.1.4/serial1612425621719/boot846ff451；planeA约120Hz正常，旧450_1仍广播导致vrpn_mocap自动发现崩溃，实参不支持trackers。仅预览改用原生planeA专用接收+ROS源，哈希/语法检查部署到/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0。18秒raw2139/preview2125，状态rejected0、无过期；同戳位置转换核对通过。domain191+localhost隔离，无fmu输入发布，无解锁/PWM/参数/网络修改，current仍b4453dd。预览与原生子进程退出，查询daemon191 PID7351已正常停止。旧SSH28664断线，新ssh-N保活43619保留。候选安装、杠杆臂、服务器时序未验收；源时间与A墙钟约差12小时，不可直接用于PX4采样时间。证据report/mocap_connection_20261019/preview_on_a_20261021T091329Z；未完成PX4发送或融合。

2026-10-21 planeA实时人工移动方向核对完成：在A机已部署隔离预览逐步采集baseline/forward/right/up/yaw_baseline/yaw_left。机体前+0.9367m（用户确认实际接近1m）、右+0.8782m伴前偏0.077m、实验室Z抬高-0.2501m；抬高有倾斜故重取水平转向基准，左转相对yaw-86.31度，最终roll0.292/pitch0.179/yaw-89.130度。四项方向与手动动作相符，六次流程均正常清理。完整姿态安装、精确尺度、杠杆臂、源时间/跟踪失效仍未验收，不改mounting_confirmed等标志。未发送PX4/改参数/运行飞行或PWM。证据report/mocap_connection_20261019/axis_check_results_20261021.json及planeA实时方向验证_20261021.md；采集状态当前yaw_left，未来新一轮baseline应显式重取。


## 2026-10-21 当日进度文档入口

已整理项目根目录 2026-10-21_A机动捕接入与实机验证进度.md，集中记录 SSH/QGC、planeA 原始接收、转换预览与 A 机六步人工方向验证、代码/部署/日志入口、未完成项及继续顺序。docs/A机动捕转换预览.md 已同步更新，移除过时的“未部署 A”状态。当前仍仅预览，未实现 PX4 VehicleOdometry 发送/融合，安装/杠杆臂/源时序确认项仍 false；方向检查通过不代表可起飞。本次仅整理文档，未操作设备。

## 2026-10-23 动捕→PX4 地面桥接实现与诊断（最新）

用户确认仍无桨/未解锁、偏移未测，随后明确暂时无法操作动捕服务器，先完成桥接和诊断验证。Ubuntu22.04新增 mocap_odometry/mocap_odometry_node/mocap_source_node、原生显式planeA接收器、config/mocap_a.bridge.yaml和独立ROS入口。默认disabled、scope=ground_only、确认项false；无真实PX4发布。支持FRD任意朝向、xyzw→wxyz、未知速度NaN；源/接收双时间、排队/倒退/过期/断流锁存，真实输出要求A身份、未解锁已着陆、稳定DDS时间同步及显式启用。它是限时地面工具，不是可直接飞行的常驻服务。

PX4只读确认1.16.0；完整1127参数前后快照一致（含类型/索引）；EKF2_EV_CTRL=0、UXRCE_DDS_SYNCT=1。源时间仍相对Jetson快约12小时并有数秒偏差，映射/不确定度未验证，不自动用到达时间猜源偏移。23逻辑+10转换回归+ROS域192localhost合成检查通过，日志report/mocap_fusion_20261023。

A独立诊断暂存到/home/nvidia/payload_catch_mocap_fusion_20261023/diagnostic_*，精确哈希看manifest。域191localhost，18秒raw1919/odometry333，同戳333对换算通过，最大算术差5.91e-8m（不代表物理精度）。出现一次原生接收间隙277642us（该处源间隔17012us），超过150ms，停发锁存正确；validation.passed=false，连续健康未通过，不能掩盖/放宽阈值。网络或进程调度根因未定位，安装/杠杆臂/服务器时序/跟踪失效仍待验收。最终本地又补严格sensor类型和真实输出竞争发布者检查，隔离回归通过，不把A历史暂存当最终发布。

没有写参数、切current、修改网络或自启、解锁/飞行/PWM。current仍b4453dd，原Agent1019和relay1023保持active；final_audit确认本轮机载进程无残留。后续先解决源时间、偏移及接收间隙，再无桨验证实际里程计输入/EKF。详见docs/A机动捕PX4桥接与诊断_20261023.md；最终参数与进程核对report/mocap_fusion_20261023/final_verification.json。

2026-10-23 用户说明服务器日期正常北京时间后，再次只读采集6秒planeA716帧：Jetson14:33:23.731+08、VRPN回调末帧按Unix解释为次日02:33:27.228+08，仍约+12小时3.497秒。该差包含停止耗时，不是精确偏移；不能据此断言服务器系统时间错，须核对服务端编码及VRPN客户端时间校正语义。Jetson NTP状态正常，配置时序确认仍false。接收PID9859正常退出、无硬件修改。证据report/mocap_fusion_20261023/动捕时间核对_20261023.md及source_time_20261023T143314/result.json。

2026-10-23 时间差进一步排查：仅读vrpn头文件并尝试被动报文/回调对照；首次sudo包装进程communicate超时，尚无有效结论，stderr保留wire_time_20261023T143955。本地修正后SSH已不可达，Wi-Fi.4/有线.101和有线.4均超时，用户确认仍上电，电脑IP仍.2/.123。未改任何时钟/网卡/路由/SSH/PX4配置，未启动融合。首次原生reader自身限30秒，但断连后未远端复核最终PID，不能宣称已确认清理。下一步先本地hostname -I核对A当前地址、只读SSH服务状态，再恢复检查。详见report/mocap_fusion_20261023/时间戳排查与连接中断记录_20261023.md。

2026-10-23 SSH已恢复：显式网卡临时socket探测后普通SSH两条路径.2→.4及.123→.101均成功，严格主机密钥与serial1612425621719一致。新boot=a006295d-9ab4-41c6-a277-f0a2455a86a8（原cbbd1ba2），uptime118.22秒，uptime-s约14:53:14；证明期间重启过，但不能据此确认14:40左右最初断线的原因。SSH/Agent/relay均active，无planeA_stream/本轮桥接采集进程残留；/32 WiFi路由仍在，电脑网卡重叠也仍在。全程只读或每socket选择网卡，未改系统路由/配置/重启。证据report/mocap_fusion_20261023/reconnect_readonly.json及per_interface_probe.json。之前“无法远端复核进程”状态已被此次新boot和进程检查覆盖；不要把路由优先级线索当唯一已证实根因。

2026-10-23继续接入最新：用户说明FZMotion（版本未知），原点不知道/尚不能测。wire_time_20261023T150023已证原始UDP字段与回调时间相同（快约12h3.5s），不归咎客户端转换；message_time_20261023T150644显示pong与pose同样偏差，ping为本地。NTP无响应；公开VRPN参考源码读取网络超时未成功，不能宣称已证应答时间语义。roundtrip_20261023T151229的20次标准ping均应答，RTT12.022–39.569ms，条件偏移交集[-43203513008,-43203501894]us（中点-43203507451、半宽5557us）；仅诊断假设，不是精度承诺，没写入配置。
重启后真实桥接复测diagnostic_20261023T070724Z：18s raw2086/odometry577，同戳577对换算通过、最大间隙36.438ms、无拒绝/锁存；passed=true仅此次样本，之前278ms根因未解决/长期未验收。默认disabled+191隔离，真实PX4发送0，源时序/原点/跟踪失效确认仍false。未改时钟/网络/PX4参数/飞行或PWM。进程正常退出、current保持b4453dd、SSH与既有服务正常。进度入口report/mocap_fusion_20261023/FZMotion接入进度补充_20261023.md及fzmotion_timing_progress.json。下一步需FZMotion版本/时间语义和planeA原点/局部轴显示，测物理偏移；不能默认中心或硬减12h启用。


## 2026-10-23 接收时间地面融合实机验证（覆盖此前“未向 PX4 发送”状态）

用户明确授权后新增 receive_time_ground 独立配置与显式 CLI 确认，原严格默认配置仍 disabled、全部未标定确认仍 false。保留 UXRCE_DDS_SYNCT=1；timestamp_sample 使用原生回调接收时刻，不修改/猜测服务器时间，不关闭 DDS 同步。限定 A 身份、未解锁/着陆、1–120秒和失效锁存；新增相对时间漂移/完全不变位姿检查，仍不能证明绝对采样时刻或所有重放，非飞行模式。FZMotion 版本已确认3.0.0.9。

本地23+9+10单元及隔离ROS通过。A真实Jetson执行 receive_ground_20261023T074805Z，临时EV_CTRL=9，仅水平位置+航向：cs_ev_pos/cs_ev_yaw真22帧跨度21.21s，末10s1000个xy/vxy均有效、dead_reckoning均false。最后桥接accepted3623/forwarded1112/rejected0/无锁存。**heading_good_for_control仍false，完整验收passed=false/exit2，未放行飞行。** 对应PX4源码FRD EVyaw启动会清yaw_align，isYawFinalAlignComplete仍要求它，与观测一致；当前labFRD非地理NED，不可只改frame数字绕过。待解决航向参考、人工移动核对融合、原点/延迟/丢跟踪，未解决电源/电池等既有门禁。

全程观测未解锁/已着陆，无飞行/电机/PWM指令；独立参数守护实际正常回退EV_CTRL=0、SYNCT=1，前后1127参数含类型索引差异0。守护45秒超时路径未故障注入实测。正式current仍b4453dd，Agent1011/relay1013持续active，机载任务进程0残留。SSH/网络/时钟/自启未改，无常驻融合。代码仅Ubuntu22.04开发，A独立暂存。报告 docs/A机动捕接收时间地面融合_20261023.md；汇总report/mocap_fusion_20261023/receive_ground_summary.json、receive_final_audit.json；恢复快照parameters_20261023T075120Z。


## 2026-10-23 航向候选地面静态测试通过（最新，覆盖“航向可控仍false”）

用户确认A无桨未解锁放稳、planeA跟踪、QGC连通，并说明无北向标定。SSH仍.123→.101/serial1612425621719/boot a006295d。只读heading_probe_20261023T081303Z采集1586对，动捕labFRD yaw约-1.274543°、PX4磁估计yaw-47.109135°，差-45.834592°/std0.024870°。候选将Rz(delta)同时用于位置和完整姿态，对齐当次PX4参考并输出NED；**north_verified=false，仅地面候选，不宣称真实北向标定，不能直接用于双机共享轴**。配置要求来源、相同旋转、同boot和10分钟启动有效期，默认严格/普通FRD配置不变。

首次heading_candidate_20261023T080951Z源间隙>150ms锁存，forwarded0/未改参数，证据保留；只调整源先预热4秒200帧，再启动桥接，阈值不放宽。第二次heading_candidate_20261023T081425Z通过：cs_ev_pos/cs_ev_yaw 22样本跨度21.210s，启用5秒后2206个位置/速度/航向可控同时有效，末10s全有效且dead_reckoning=false；桥接accepted3685/forwarded1117/rejected0。**静态候选通过不是飞行放行或动态/绝对方向验收**；启动间隙根因及长期稳定仍未证明。新增6航向单元及ROS位置姿态一致/过期拒绝/解锁停发，通过原23+9+10回归。

全程观测未解锁着陆、无飞行/PWM指令。EV_CTRL临时9后已自动恢复0，SYNCT始终1，1127项参数数值类型索引前后0差异；current仍b4453dd，Agent1011/relay1013正常，任务进程0。没有常驻融合。最新完整参数parameters_20261023T081620Z；审计receive_final_audit.json。

保存前移基准heading_movement_baseline.json：raw约[1.8785946,-.1999071,.2916852]，候选EV约[1.2082929,1.4522768,-.2916850]，PX4约[1.2080616,1.4526118,1.1031359]，yaw-.8219225rad。高度仍气压计且静止长期明显漂移，**不要把PX4 z与EV z当同基准或宣称高度验收**。下一步先人工前移/右移/转向，先比逐姿态融合位移再记录连续动态；若快照超时/重启须显式重采同基准，不能混参考相减。其后高度、刚体偏移、时延、遮挡断流/保护和持续飞行桥接，供电false/电池等既有门禁仍待解决。报告docs/A机航向参考与静态融合验证_20261023.md；汇总heading_candidate_summary.json，候选完整配置在通过run目录candidate_config.yaml。


## 2026-10-23 前移59.7705厘米端点检查：磁航向参考跨位置/时间不稳定（最新）

用户回复前移约59.7705cm。A同boot/serial、SSH.123→.101正常。本轮**只读**heading_probe_20261023T082553Z，1601配对，末半801raw位姿与heading_movement_baseline对比：总位移60.0873cm（比用户值+0.3168cm），按原机头水平分解前+59.8868cm/左4.9057cm，up约0.0072cm，相对yaw+1.7587°。方向基本符合；不是独立精度或连续运动验收。

发现关键新问题：前后动捕yaw -1.274543°→+.483709°，PX4 -47.109135°→-86.288412°（变化-39.1793°），候选参考旋转 -45.834592°→-86.772121°，相对参考变化-40.9375°。当前mag_heading_consistent=false、heading_good=false、EV_CTRL0且不在融合。中间有上次临时融合/恢复，不能断言全部是移动本身/磁干扰所致；但**磁航向快照不能作为已稳定标定，静态通过不等于跨位置/时间可用**。

没有重采新磁偏移来启用融合、没有更新旧时间冒充新参考。run_heading_candidate_on_a.py增加已有基准与新旋转>5°本地拒绝，实测-40.938°在任何SSH暂存/参数写前拒绝。离线--with-fusion也拒绝漂移；未修改核心桥接或门禁。只读完整1127参数与上轮恢复后差异0，EV_CTRL0/SYNCT1、任务进程0，current b4453dd/Agent1011/relay1013正常，无硬件/网络/时钟改变。源子进程正常退出。

下一步先保持当前位置，不继续右移，把航向参考跨位置/时间变化排查清楚并建立固定独立参考，不能每个位置重设世界轴。此前“静态三标志通过”为单次事实，不是可靠北向/飞行标定完成；本次尚未通过PX4融合位移验证。高度、偏移、时延、失效保护仍待。文档docs/A机前移验证与航向参考漂移_20261023.md；证据forward_597705_summary.json、forward_final_verification.json、forward_final_audit.json、parameters_20261023T082943Z。保留原heading_movement_baseline未覆盖，不把新点假称已融合基准。


## 2026-10-23 固定动捕航向/MAG_NONE实机静态验证通过（最新）

用户换电后继续授权无桨未解锁地面融合。A序列号1612425621719/Jetson boot39417b19保持，SSH .123→.4。实现独立fixed_lab_navigation配置，固定[Y,X,-Z]、world原点0和完整姿态同转；明确非地理北向的室内导航NED编码，fresh参数证明MAG5/EV9/SYNCT1、同boot与UID、磁/地理观测冲突即停发，不复用磁航向快照。PX4同版EKF库7场景、本地5固定+既有23/9/6/10及ROS、4事务/1重启证据/2批量证明验证通过。

最终fixed_reference_20261023T094649Z机载passed=true：MAG5/EV9重启读回，静态25秒，EV位置+航向标志28样本跨度26.270秒，2627个位置样本xy/vxy/heading_good_for_control均true、dead_reckoning均false；磁航向/3D均false，真实输入1035帧。软件停止源后400ms无新输入并锁存，物理遮挡未测。先前4轮失败保留：约82秒飞控网络恢复超过早期时限、连续重启uptime判断、跨重启旧订阅、串行参数证明过期；最终修复且未放宽2秒证明过期/未解锁着陆门禁。

结束EV0/MAG0/SYNCT1已恢复并再次重启飞控核验。独立after_20261023T095131Z1127参数读全，仅CAL_BARO0_OFF -48.375→-38.1484375变化，符合PX4启动自动相对气压计校准源码；不是全参数完全一致，不强行覆写。Agent1006/relay1008持续active、current仍b4453dd、本任务机载进程0，未改网络/时钟/自启，未解锁/飞行/电机/PWM。核心目标已在静态通过；当前不常驻融合。人工连续移动/转向、精确安装偏移/源延迟/物理遮挡/高度/飞行桥接及失效策略仍待，不能用地面工具直接飞行。报告docs/A机固定动捕航向实机验证_20261023.md；证据report/fixed_reference_20261023/final_summary.json。所有本轮文件留在Ubuntu22.04工程，旧工程及C盘无新辅助文件。


## 2026-10-23 动捕Z参考实机静止融合通过（最新）

用户明确要求“你来融合z”。新增config/mocap_a.xyz_ground.yaml及显式height_fusion=external_vision_reference/height_origin=固定实验室marker原点非AGL；核心XYZ路径严格要求MAG5/EV11/HGT3/SYNCT1同boot新鲜重启证明，旧XY路径不放宽。位置仍[Y,X,-Z]、固定world原点0，未伪造安装/时序/遮挡标定。PX4同版EKF8场景、高度5/三参数回退5/既有23+9+6+10及隔离ROS通过。

A机xyz_reference_20261023T101442Zpassed=true，静态25秒；EV位置/航向/高度标志共同有效跨度26.270秒，XYZ及速度/可控航向有效、磁航向/3D关闭。2000组稳定日志Z输入/估计差均值0.000641m/最大0.001618m，仅静态数据接近不是测量精度。软件断流400ms后无输入并锁存，物理遮挡与人工抬高/动态未测。

测试后EV0/HGT1/MAG0/SYNCT1已恢复、飞控重启核验通过；完整1127参数独立审计after_20261023T101935Zpassed=true，差异{"CAL_BARO0_OFF": {"before": {"value": -38.1484375, "type": 9, "index": 59}, "after": {"value": -47.1484375, "type": 9, "index": 59}}}，自动校准候选保留，不宣称参数全部相同。Jetson未重启、Agent/relay与current不变、本轮进程0、未解锁/飞行/执行器/网络时钟自启修改。当前没有常驻XYZ融合。后续人工抬高/移动、起降平面与IMU偏移/源延迟/遮挡/飞行桥接及失效保护待。报告docs/A机动捕XYZ与航向融合_20261023.md，证据report/mocap_z_20261023/final_summary.json。


## 2026-10-23 人工抬高连续融合验证（最新）

用户确认无桨未解锁水平放稳/planeA跟踪无牵扯，执行lift_reference_20261023T102520Z。结果{"ground_test_passed": false, "post_audit_passed": true, "flight_approval": false, "run": "lift_reference_20261023T102520Z", "post": "after_20261023T103137Z", "phases": {"baseline": {"label": "baseline", "monotonic": 6129.477717295, "input_xyz": [0.39766841325908897, 1.8219980359077455, -0.2917708531022072], "px4_xyz": [0.396642729267478, 1.8217230749130249, -0.2918688065558672], "px4_z_span_m": 0.0008093118667602539, "input_z_span_m": 0.00013631582260131836, "input_yaw_deg": 2.5246609006182212, "px4_yaw_deg": 2.5301278201986133, "local_samples": 400, "input_samples": 160, "flags_samples": 3}}, "lift": null, "error": "RuntimeError('manual interaction window expired')", "all_disarmed": true, "all_landed": true, "restored": {"changed": true, "restored": true, "restore_restart_verified": true, "flight_approval": false, "flight_commands": 0, "fc_only_reboots": 2, "lease_ended": true}, "parameter_differences": {"CAL_BARO0_OFF": {"before": {"value": -47.1484375, "type": 9, "index": 59}, "after": {"value": -48.21875, "type": 9, "index": 59}}}, "remaining_task_processes": [], "passed": false}
原landed/未解锁/新鲜时间及参数门禁保留，PX4源码未解锁landed强制true，不代表手持飞机物理接地。未解锁/执行器/网络时钟自启修改；当前测试结束不常驻融合。详见docs/A机动捕高度抬高验证_20261023.md。


## 2026-10-23 用户结束今日工作（最后状态）

用户暂不能抬高，随后明确先结束、保存今日成果并退出工作状态。手动验证只完成baseline，原始程序因手动窗口到期退出，EV0/HGT1/MAG0/SYNCT1已恢复且飞控重启核验；after_20261023T103137Z独立1127参数审计通过，仅自动气压计校准候选差异，本轮机载进程0。已完成XY/yaw与XYZ/yaw静止融合仍有效，动态抬高待测，当前没有常驻融合。Jetson/SSH/网络及既有服务未变，没有自动继续任务。成果快照及继续入口report/session_close_20261023，用户再次要求继续前不再设备操作。


## 2026-10-23 用户授权整理工作区与 Git

用户要求整理干净。完成本地成果检查、源码快照与 SHA256 清单，原始采样、日志、历史备份及现场凭据辅助脚本保留原处并精确忽略；源码、配置、部署入口、文档和顶层关键结果纳入本地版本保存。report/README.md、ARTIFACTS_20261023.json 与 docs/工作区与Git管理.md 说明边界；原始归档不会随 Git 克隆获得。没有删除/移动成果，没有推送或连接无人机，没有改变已部署版本和飞控状态。

离线 quick 检查47组全通过；暂存内容Python305/Bash59/YAML13/JSON141语法及解析检查通过，diff格式检查通过。仅移除一处GUI源码行尾空格和一份历史诊断脚本末尾空行，原字节已在本地源码ZIP保存、索引注明。3047个原有文件SHA256一致；原始采样/日志未变。完整核验日志在report/workspace_cleanup_20261023，未启动后台进程或常驻服务。该版本保存不代表动态抬高验证/飞行放行完成。


## 2026-10-26 有线恢复后修复Wi-Fi SSH（最新）

用户授权接电脑网线配置Wi-Fi，要求保留飞控功能。有线eno1=10.41.10.100身份serial1612425621719已核对，boot3f7e18cb。电脑Wi-Fi保持DHCP .4/22与自动DNS；只临时给电脑有线加10.41.10.50（无gateway/DNS、保留原.123），已删除恢复。A当前Wi-Fi .5/22；原Mesh /24抢回程是已确认原因，内核ip rule不支持。新增payload-catch-wifi-peer.service（enabled/active，v3）按电脑Wi-Fi MAC98:5f:41:64:80:ed被动ARP识别并维护唯一proto186/metric9 /32 Wi-Fi路由，支持同网卡DHCP地址变化，保留其他路由/配置；识别不是身份安全边界，SSH仍严格旧主机密钥。9项离线测试通过，明确强制Wi-Fi接口SSH及普通SSH均成功。电脑实际DHCP变化/整机重启尚未实测；A自身DHCP地址也可能变化。

用户确认飞控原网线接回后10.41.10.2 ping2/2，12秒被动接收5844个飞控UDP包（含8888）；原Agent1155/relay1156未重启，7个原配置文件哈希和原路由不变。没有飞控参数/指令、解锁/PWM/系统重启/原服务重启。原本Masked的microxrce-agent/mav-relay不应启动；实际运行的是managed路径，保持原状。current仍b4453dd。原始验收日志report/ssh_connection_20261026，说明docs/A机WiFi_SSH修复与回退_20261026.md。精确撤销：sudo python3 /home/nvidia/payload_catch_network_backup_20261026_v3/transaction.py rollback。旧两次失败安装已回退，最终仅v3有效；验证后自动回退计时器inactive。退出SSH不停止保留的网络辅助服务。没有其他常驻采集或SSH会话。


## 2026-10-26 动捕XYZ与航向实机续验（最新）

用户确认无桨未解锁水平放稳，Wi-Fi .4→A .5，serial1612425621719/boot3f7e18cb保持。8秒原生planeA960帧、约120Hz/invalid0/maxgap21.518ms，源时间快约12h3.11s，继续明确receive_time_ground。复用10月23有界地面工具，只改当前IP/日志，不放宽门禁。1127参数先备份，临时MAG5/EV11/HGT3/SYNCT1和飞控重启核验；静态XYZ/航向共同有效93.04s、local9303、input3727、flags92，不等于动态或绝对精度。

人工抬高询问后用户回答暂时无法完成；仅baseline，未抬高/放回，原始passed=false/RuntimeError('manual interaction window expired')，正常退出信号按精确进程发送。EV0/HGT1/MAG0/SYNCT1已回退重启读回，独立1127参数审计passed=True；仅CAL_BARO0_OFF自动校准候选差异，原始详情保留。Jetson/current/Agent1155/relay1156不变，本轮进程0残留，不常驻融合、无解锁/飞行/PWM/网络修改。文档docs/A机动捕XYZ融合续验_20261026.md；完整证据report/mocap_continue_20261026/final_summary.json。动态手动验证仍待，下一次准备好连续动作后重新采基准再测。


## 2026-10-26 自动抬高片段记录（最新）

自动有界独立机载任务lift_reference_20261026T031255Z保存baseline/elevated；动捕上升0.700788m、PX40.699714m，差0.001074m，物理距离未独立测量。用户已抬高/随后放回，但returned未捕获，96秒窗口超时，原passed=false，不得记全程通过。参数EV0/HGT1/MAG0/SYNCT1恢复重启核验；after_20261026T032116Z1127参数独立审计通过，仅CAL_BARO0_OFF自动校准候选差异，进程无残留，current/Jetsonboot/原服务不变。证据report/mocap_auto_lift_20261026/final_summary.json，说明docs/A机自动抬高片段验证_20261026.md。下一步每轮只采单个动作，避免提示延迟使往返超时。


## 2026-10-26 用户暂停去吃饭，保存验收进度（最新）

用户明确先验收到这里并保存进度，去吃饭；后续明确要求继续前不再连接/操作A，也不自动启动测试。静态XYZ/yaw93.04秒通过；自动抬高片段约70.08cm vs69.97cm，返回未采到，不是整轮通过。左转run=lift_reference_20261026T032518Z自动检测exactly repeated pose stream，桥接停发、原passed=false，没有合格left_turn稳定段。用户确认反光点可见且未观察到跟踪丢失，原因未定，不写成已证遮挡，不放宽阈值。端点诊断input变化-80.985°/PX4-83.806°，仅点态响应证据。
after_20261026T033215Z独立1127参数恢复审计通过，EV0/HGT1/MAG0/SYNCT1；仅CAL_BARO0_OFF自动校准候选差异，原始详情保留。Jetsonboot/current b4453dd/原Agent1155和relay1156不变；采集/源/桥接/守护无残留，当前没有持续融合。保留此前用户要求的Wi-Fi回程辅助服务，不留监视或SSH保活。
集中进度docs/A机动捕PX4验收进度与继续入口_20261026.md，根目录2026-10-26_A机验收进度.md；转向汇总report/mocap_yaw_20261026/final_summary.json。下一步优先离线排查重复位姿及源时间语义，再经用户继续后做单动作左/右转与真实融合平移/抬高放回；精确安装偏移、源延迟、物理遮挡恢复与飞行桥接仍待。未提交/推送/新部署。


## 2026-10-26 午后水平左转单动作融合通过（最新）

用户午后明确继续，确认无桨/未解锁/水平放稳并可配合；A用户重启后boot=dc981513-5789-4a7c-ba91-57367486bfa9，Wi-Fi .4→.5恢复，Agent1019/relay1024/网络辅助2770。仅只读VRPN三轮各约120秒，静态、左转约89°和右转约55°均保存，无100ms以上完全重复位姿；午前83帧重复仍无根因，不放宽门禁。

融合重试055101Z在窗口内原始姿态变化不足0.11°，没有合格转向，原passed=false保留；用户称FZMotion模型跟随，原因未确定。再次实时原始右转后复验lift_reference_20261026T055930Z，baseline/left_turn已保存，validation passed=true，动捕变化-61.930407°、PX4变化-61.720678°、差0.209729°。基准后61个flags样本均EV XYZ/yaw真、mag融合假，无锁存故障。只通过单次左转地面动作，物理角度未独立测量；转向稳定段水平位置均值差约3.80cm仍待专门平移/动态核对，不宣称全部动态精度通过。

EV0/HGT1/MAG0/SYNCT1恢复重启核验，after_20261026T060444Z独立1127项审计passed=true；仅CAL_BARO0_OFF自动校准候选差异保留。Jetson boot/current/三服务PID保持，任务进程无残留，不常驻融合或SSH保活；没有解锁/执行器/飞行/网络或时钟修改。证据report/mocap_resume_20261026/final_summary.json，更新根目录和docs验收总表。右转真实融合、前/右平移、抬高放回全程、时间/安装偏移/失效保护及飞行桥接仍待；本次未提交/推送/新部署。



## 2026-10-26 右转和前移停稳段真实融合通过（最新）

本段为前述午后左转之后的新增进度，先前失败与限制仍保留。

| 项目 | 本轮结果 | 数据 |
| --- | --- | --- |
| 右转真实融合 | 通过 | 动捕58.269042°，PX4 57.832236°，变化差0.436806°；基准与right_turn稳定段均保存 |
| 前移真实融合停稳段 | 通过 | 动捕向前0.764463m，PX4向前0.765243m；水平位移向量差0.829mm，停稳端点差0.335mm |
| 真实融合向右平移 | 尚待 | 右转不等于右移 |
| 抬高后放回全程 | 尚待 | 先前仅抬高稳定片段，返回稳定段未验收 |
| 飞行动态精度/时延 | 尚待 | 本轮均值与停稳一致性不代替运动全过程误差或时延标定 |

右转稳定段水平位置均值差约6.68cm，原因未确定。前移专门等待输入位置/航向稳定、额外保持8秒，再取4秒均值，停稳后位置已对齐。本轮前移约76.45cm，伴随横向偏移-12.17cm；未独立测量实际距离，不宣称动捕具有亚毫米绝对精度，也不由停稳相符认定转向时水平误差问题已解决。最近ROS接收时间配对的误差诊断单独保存，不冒充同步采集时刻比较。

两轮均保持XYZ/yaw融合启用、磁融合关闭；本轮原始数据没有持续100ms以上完全重复位姿，没有锁存桥接故障。午前83帧重复根因仍未查清，原保护阈值未放宽。

两轮均已恢复EV0/HGT1/MAG0/SYNCT1，飞控恢复重启核验，完整1127参数独立审计通过；额外差异仅CAL_BARO0_OFF自动校准候选，完整原值在各comparison.json，未强行覆写。Jetson boot、正式部署链接、Agent1019/relay1024/网络辅助2770保持，任务进程无残留，不常驻融合，没有解锁/电机/PWM/网络更改。

证据：
- report/mocap_right_20261026/final_summary.json；成功run=lift_reference_20261026T061125Z，审计=after_20261026T061628Z。
- report/mocap_forward_20261026/final_summary.json；run=lift_reference_20261026T061834Z，审计=after_20261026T062423Z。
- 各run保留samples.jsonl、baseline、动作稳定段、参数前后快照、运行清理和nearest_receive_time_comparison.json。



## 2026-10-26 向右平移真实融合（最新）

用户指定验证向右移动。本轮lift_reference_20261026T064203Z，地面动作验收**通过**。

首轮062808Z在基准前因验收脚本误将转发计数字段forwarded写为righted而中止，未发出移动提示。该误改已修正，五组方向投影检查通过；原失败记录与独立恢复审计保留在retry_correction.json，不改写为成功。该首轮只修验收工具。第二轮063332Z完成baseline后，因fixed reference parameter readback stale/future锁存停止，未保存合格right_move；完整日志与独立1127参数恢复审计保留。本次另修项目桥接mocap_odometry_node.py的参数证明校验顺序：读完文件后再获取单调时钟，避免并发刷新时将新证明误判为未来。旧顺序竞态已离线复现，7项检查通过，实际未来/超时/身份/磁融合冲突及锁存保护保留。现场第二轮缺少故障瞬间证明时间，不能断言唯一根因已证实；本轮成功不抹去此前失败。2秒证明时限、源时间与新鲜度门禁、冻结保护均未取消。修复在本地源码及本轮独立测试目录，正式current未切换。

动捕向右90.900cm，PX4向右90.841cm；水平位移向量差1.879mm，停稳端点差1.626mm，航向差0.219°。伴随沿基准机头方向偏移-3.035cm。

按基准时机头方向的右侧计算正向位移，保持原先的位置/姿态新鲜度、未解锁、XYZ/yaw融合和无磁融合门禁，保留冻结保护及完整参数回退。输入位置/航向稳定后额外保持8秒再保存4秒均值。物理距离未经独立测量；毫米差值只表示输入与估计在停稳段的一致性，不是绝对定位精度或运动全过程/时延验收。基准后观测72个融合状态样本，共同融合是否全部有效：True；桥接锁存故障数：0；原始完全相同位姿持续100ms以上的片段数：0。

本轮参数EV0/HGT1/MAG0/SYNCT1已恢复且重启核验；after_20261026T064755Z/comparison.json独立1127参数审计通过。额外差异仅气压计自动校准候选CAL_BARO0_OFF，详情{"CAL_BARO0_OFF": {"before": {"value": -50.65625, "type": 9, "index": 59}, "after": {"value": -54.9296875, "type": 9, "index": 59}}}。原正式部署、Jetson boot、Agent/relay/网络辅助服务不变，任务无残留，无持续融合，无执行器动作或网络修改。

用户已明确：**抬高本身已有成功验证**（约70.08cm vs69.97cm），无需重新记作未完成；只剩放回后的稳定段/全程尚未验收。已有静态、左转、右转、前移的成功记录保留。向右移动本轮状态见本段，不再混同右转。

下一步仍需处理源时间/时延、安装偏移、午前冻结根因、物理跟踪失效与恢复、飞行桥接及接管/起飞条件；地面动作通过不放行飞行。

证据：report/mocap_right_translation_20261026/final_summary.json、lift_reference_20261026T064203Z/validation.json和samples.jsonl、基准/动作稳定段与参数前后快照。没有提交、推送或新部署。

## 2026-10-26 持续动捕-PX4地面融合已启动

用户要求直接融合 PX4 与动捕。本次手动会话已在 A 机运行：payload-catch-mocap-ground-20261026t070536z，正式 current 未切换、无开机自启。EV位置/高度/航向融合启用，磁融合关闭，持续门禁要求 A 身份正确、未解锁、已着陆、动捕新鲜且单一发布者。实时检查已见动捕转发、PX4 EV XYZ/yaw 有效、磁融合关闭、flight_approval=false。

这是无桨地面会话，不是飞行放行。安全停止脚本位于 report/mocap_continuous_20261026/stop_continuous.py；停止后守护恢复 EV0/HGT1/MAG0 并重启核验。证据与会话说明见 docs/A机持续动捕融合地面会话_20261026.md。

## 2026-10-28 本对话持续适用的工作原则
用户要求：入口只校验一次，避免重复判空和滥用 try/catch；单次逻辑不乱抽函数，低概率防御先征得同意。坚持最小改动，避免标识符、分支、临时文件和构建流程堆叠，以 A 机尽快完成空载首飞为主线。不要自行增加重防御、额外门禁或无关构建；必要的定位新鲜度、失效处理、飞控自检和人工接管按最小实现保留。用户已取消无桨实体遮挡/断流测试作为首飞前验收项。


## 2026-11-01 恢复工程后本地Git保存与阅读（最新）

用户要求先更新提交Git并阅读工程；随后明确“更新提交git，阅读完工程就行”，本轮以此为最终范围，不继续无人机操作。恢复成果检查点a218967已保存；飞行桥接旧测试与10月28日显式接收时间配置不同步已修正，新增缺失确认拒绝测试，83项相关离线检查全部通过。生产控制/融合代码和机载参数未改。用户说A已上电、动捕暂不可用；仅一条SSH hostname只读命令成功，未启动扩展机载检查、融合、任务或执行器，远端fetch等待取消。此次只做本地版本提交，不宣称远端已更新。README/AGENTS/MEMORY、ROS节点入口、部署草稿和最近验收记录已阅读，后续待用户新指示。
