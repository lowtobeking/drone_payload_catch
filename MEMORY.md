# MEMORY.md — 项目记忆（给下一个 AI / 未来的自己）

> 最后更新：2026-10-05。**新对话请先读本文件**，再按需读 `README.md`、`report/`。
> 目标读者：接手本项目的 AI 助手。读完应能直接继续干活，不必重跑全部排查。

---

## 0. 一句话现状

「无人机 A 空投载荷、无人机 B 空中接住」项目。**离线算法层完整**（M0/M2/M3/M3+/M4 + 大量鲁棒性探索，含多组对照与负结果）；
**SITL 层跑通并已加固**（PX4-1.16 双机 + Gazebo 真实载荷），当前覆盖：

- **场景**：M1 水平会合；M6 垂直堆叠（离线 200/200；SITL 捕获→携带→双机分开落地）；
  **M6-moving 编队同速（支持速度 0.5/1.0 m/s，各 2/2 完美）**；
- **末端能力**：大漏斗 `x500_funnel_big`（捕获余量×14）、空心导向锥杯 `x500_funnel_cup`、
  **主动保持锁扣** `payload_lock`（接住→刚性携带→落地，绕开 `v_retain`）；
- **抗扰**：速度前馈/预测对正、增广风 KF、A 端迎风预补偿、自适应下潜；
- **协同**：释放握手（B 报就绪→A 释放权威→ack）、意图（预测落点）、时钟同步、编队握手、
  **及时释放 + 释放超时中止**；
- **安全**：不确定度 keep-out、释放后清场、**飞行终止(kill)** 监督；
- **控制/规划**：ZEM 终端导引、释放前落点余量闸。

**未做**：真空心漏斗（现为 primitive 杯/实心盘）、真实夹爪/磁吸、真机、真实相对导航、平台 failsafe 边界（~4.9 m/s）。

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
├── config/catch_scenarios.yaml← 单一真值源（defaults/layouts/scenarios/thresholds）
├── payload_catch/
│   ├── payload_model.py       ← 载荷抛体模型（解析；可选阻力/风）
│   ├── rendezvous.py          ← 会合规划：min-energy 三次、软终端速度、过冲惩罚、分段参考、solve_inflight
│   ├── sim_core.py            ← 离线闭环仿真（A 恒速飞行 + B 控制 + 捕获 + 闭环重规划）
│   ├── mpc_terminal.py        ← B 的 acados 终端 MPC（含指纹缓存）
│   ├── payload_filter.py      ← 载荷状态估计（KF / 朴素）
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
│   └── mc_m6_sitl.sh          ← M6 SITL 蒙特卡洛（每档 N 次换种子，输出 report/m6_sitl_mc.md）
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
    ├── planning_control_opt.md ← **规划/协调+控制优化**：ZEM 终端导引 + 释放前落点余量闸
    ├── safety_control_review.md ← **保护控制审查**：已有(限幅/keep-out/释放闸) vs 缺口(geofence/看门狗/abort/避碰)
    ├── safety_supervisor.md    ← **安全监督+飞行终止(kill)**：/safety/kill_a|b + 异常自动 kill；SITL 验证
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
python3 tools/offline_run.py --all              # 13 个工况，全部 PASS
python3 tools/offline_run.py --scenario M4_high_kf --mc       # 蒙特卡洛
python3 tools/offline_run.py --scenario M2_line_v10 --compare # 开环 vs 闭环

# M6 垂直堆叠投放（A 正上方释放 + B 温和下潜 + 刚性漏斗）
python3 tools/stack_run.py                      # 单次
python3 tools/stack_run.py --sweep-dive         # 扫 B 下潜加速度
python3 tools/stack_run.py --sweep-gap          # 扫 gap
python3 tools/stack_run.py --mc 200             # 蒙特卡洛，200/200
```

### 4.2 SITL（2 机 + Gazebo 载荷）
```bash
source ~/drone_payload_catch/env.sh
bash ~/drone_payload_catch/run_m1_sitl.sh 50          # PD 控制器
CTRL=mpc bash ~/drone_payload_catch/run_m1_sitl.sh 50 # acados MPC
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
| 双机协调 | `rendezvous.solve_cooperative` + 协议分析（`report/coordination.md`） | ✅ 现状=单向/A被动；改进=握手/意图/时钟/安全；⚠️**负结果：A 小幅释放偏移与 t_r 冗余（δ*=0）**，协同价值在协议与 A 的速度/高度配合 |
| 协同释放握手 | `coord_mode=handshake`（`a_node`/`b_node` + `/drone_b/ready` + `/drone_a/release_cmd` + `/drone_a/intent` + 时钟同步 `/coord/ping|pong` + 释放门限）（`report/coordination_handshake.md`） | ✅ **SITL：B 报就绪→A 作释放权威（精确自身状态）→ack→B 下潜→`STACK CAPTURED`**；含往返**时钟同步**(offset≈0/rtt0.5ms 换算 t_rel) 与释放前一致性门限；默认 direct 保留 |
| 编队握手+意图 | M6-moving FORMATION 握手 + `/drone_a/intent` 升级为**预测落点**（`use_intent`/`WIND_EST`）（`report/coordination_handshake.md`） | ✅ SITL：`/formation/start`→FORMATION→**A 释放权威**→DIVE→`STACK CAPTURED horiz=0.040m`；意图落点(风漂移) SITL no-op 验证；**ack 改为原子事件**（不再依赖瞬时对齐） |
| 安全层 + PX4风 | keep-out=`min_ab_gap+kσ`（在线估 σ）+ 释放后 A 定向清场；`use_px4_wind`（订阅 `/fmu/out/wind`）（`report/coordination_handshake.md` §4d） | ✅ SITL：`SAFETY_FLOOR=0.2` 时 `WAIT_A clear=1.37≥gap_eff=1.2`、B 保持更低、`A: 开始定向清场`、`STACK CAPTURED horiz=0.010m`；PX4 风 hookup 不报错（x500 无空速，风估计可能恒 0，`wind_est` 作兼底） |
| 协同协议验证 | `tools/validate_coord.py`（`report/coordination_validation.md`） | ✅ 3 组配置（标称/噪声+安全/编队）**全部通过**：捕获、就绪→释放→ack 顺序、单次释放、min\|A−B\|≥1.0m、无 failsafe |
| 规划/协调+控制优化 | ZEM 终端导引(`zem_gain`) + 释放前落点余量闸（`report/planning_control_opt.md`） | ✅ 离线：ZEM 临界风 w4 28→36/40、w5 13→22/40（`zem≈1`）；SITL `COORD=handshake ZEM=0.8` → `STACK CAPTURED horiz=0.055m`、无 failsafe |
| 安全监督(kill) | `px4_iface` 安全监督：`/safety/kill_a|b` 外部 kill + 异常(姿态/越界/状态超时)持续自动 kill（`MAV_CMD_DO_FLIGHTTERMINATION`）（`report/safety_supervisor.md`） | ✅ SITL：发 `/safety/kill_b` → B `Flight termination active`（动力切），**A 不受影响**（计数 0） |
| 优化第二轮 | 安全分级状态机(OK/HOLD/PULLBACK/LAND/KILL) + 越界回拉；DIVE 加速度前馈(PX4 `trajectory_setpoint.acceleration`)；B 在线 σ 共享给 A 释放闸；释放提交窗口+lead 窗口取消；3D 反应式 keep-out；在线自适应下潜（`report/opt_round2.md`） | ✅ 单元+多轮 SITL：标称 `STACK CAPTURED horiz 0.008–0.080m`、双机落地、无 failsafe；越界 `PULLBACK`、丢状态 `HOLD`（自愈） |
| 优化第三轮 | 安全硬化：安全指令绕过 `sp_rate_limit`；围栏改 `_fence_velocity` 软限幅（保留切向、消除与任务互顶）；`safe=` 进周期日志（`report/opt_round3.md`） | ✅ 单元 5 项 + SITL（围栏 3m 稳定、标称 `horiz=0.018m` 无 failsafe） |
| 优化第四轮 | 传感器/估计器约束（零新硬件）：用 `vehicle_local_position` 未用字段 — `eph/epv` 作 σ、健康/一致性看门狗(`dead_reckoning`/valid/`reset_counter`)、估计器限值(`vxy_max/vz_max/hagl_min`)（`report/opt_round4.md`） | ✅ 单元 5 项 + SITL（启动期 `estimator_reset`→1s HOLD 自愈、标称 `horiz=0.034m` 无 failsafe） |
| M6-moving 加速度 | 编队控制优化（死推算参考+稳释放门限）+ 空心杯保持 + **及时释放/超时保护** + 多重复验证（`report/m6_moving_speed.md`） | ✅ **支持速度 0.5/1.0 m/s 各 2/2 完美**（捕获+保持+双机落地、无 failsafe）；✅ **及时释放**（编队 ~3.5s 即捕获，之前 10–20s）+ **超时中止**（不投、A 保留载荷、双机安全落地）；**2.0 m/s 保留、不再优化** |
| M6-moving | 编队同速投放（同一投影点→同向同速巡航→运动中释放，物块继承 A 速度） | ✅ 无窗口 SITL 3/3（`STACK CAPTURED horiz 0.108–0.128m`，物块随漏斗落地）；GUI 偶发平台 failsafe |
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
  大漏斗 `eff_r=0.25m`；主动保持锁扣（下击暴流 8 m/s 离线 100/100）。
- **已知边界**：常规下 `v_p≈4.9 m/s` 触发 B failsafe（飞控/传感器，非算法）；
  **2.0 m/s 编队保留、不优化**；敞口杯倾斜 >25° 会滚出（不如高摩擦平盘 45°）。
- **未做**：真空心漏斗 / 夹爪 / 磁吸；真机；户外/RTK；真实相对导航（现为真值+噪声）。

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
