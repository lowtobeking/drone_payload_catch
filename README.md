# drone_payload_catch · 空投—空中捕获

> 🧠 **新接手请先读 [`MEMORY.md`](MEMORY.md)**（项目记忆：现状/环境事实/坑/下一步），再看本文件。

无人机 **A** 携载重物飞行，在算法求出的**释放时刻/会合点**抛投；无人机 **B** 实时机动，
在空中的**空间会合点**接住重物（**位置必须到达，相对速度尽量小**）。

复用既有 PX4 + ROS 2 + acados 栈的坐标约定、QoS/话题命名、`yaml` 单一真值源、安全滤波
与诊断工具。核心算法层（载荷抛体 / 会合规划 / 离线闭环仿真）**刻意不依赖 ROS**，
可在任何机器上一条命令验证。

> 坐标：世界系 **NED**（x=北, y=东, z=下），高度(离地) = `-z`。

## 当前能力速览（近期新增）

> 完整现状/记忆见 [`MEMORY.md`](MEMORY.md)；逐项日志见 `report/`。

- **末端能力**：大漏斗 `x500_funnel_big`（捕获余量×14）、空心导向锥杯 `x500_funnel_cup`、
  **主动保持锁扣** `payload_lock`（接住→刚性携带→落地）。
- **抗扰**：速度前馈 + 预测对正、增广风 KF（`BallisticDragKF`）、A 端迎风预补偿、自适应下潜。
- **安全**：不确定度 keep-out（`min_ab_gap+kσ`）、**3D 反应式 keep-out**、**分级安全状态机**
  （`OK/HOLD/PULLBACK/LAND/KILL` + 越界回拉）、释放后清场、**飞行终止(kill)** 监督。
- **协同**：释放握手（B 报就绪→A 释放权威→ack）、**释放提交窗口 + lead 窗口取消(abort)**、
  **B 在线 σ 上传**、意图（预测落点）、时钟同步、编队握手、及时释放 + 超时中止。
- **控制/规划**：ZEM 终端导引（`zem_gain`）、**DIVE 加速度前馈**、释放前落点余量闸、
  **在线自适应下潜**、速度指令速率限幅。
- **M6-moving 支持速度 0.5/1.0 m/s（各 2/2 完美：捕获+保持+双机落地，无 failsafe）**。
- **研究（协调方向）**：`report/research_roadmap.md`（C1–C5 贡献 + 差距 + 实验方案）；
  **协同交接基准（阶段 0）** `tools/bench_coord.py` → `report/coordination_benchmark.md`。

## 任务与算法

**A 的运动**（第一版）：恒速直线 `p_A(t)=a_init + a_vel·t`；`a_vel=0` 即悬停释放。
释放时载荷继承 A 的速度，随后做抛体运动：
```
p_p(τ) = p_r + v_r·τ + ½g·τ²,   v_p(τ) = v_r + g·τ      (τ = t − t_r)
```

**协调搜索**：规划器在 `(t_r, τ_c)` 上取使代价最小者
```
J = w_time·t_r + w_accel·(峰值加速度/a_max) + w_vel·|Δv|²
```
并用约束过滤：`|v|≤v_max`、`|a|≤a_max`、离地余量、捕获高度区间。

**B 的会合轨迹**：双积分器、两端位置/速度、`min∫|a|²dt` 的解析解（每轴三次多项式）。
若"终端速度精确匹配"不可行（如载荷速度超 B 限速），退为**终端速度软代价**解析解
（位置仍硬到，速度按权重折中）。

**闭环重规划（M3）**：载荷离手后，每 `replan_dt` 用**当前观测的载荷状态**重解会合
`solve_inflight(p_p, v_p, p_B, v_B)`，抵抗释放误差与模型失配；B 跟踪滚动更新的参考。

**捕获判定**：`|p_B−p_p| < r_c` 且 `|v_B−v_p| < v_c`（用载荷真值；测量噪声只影响 B 的规划）。

## 快速开始（离线，不需要 ROS/SITL）

```bash
cd ~/drone_payload_catch

# 单元自测
python3 -m payload_catch.payload_model
python3 -m payload_catch.rendezvous
python3 -m payload_catch.sim_core

# 体检报告
python3 tools/offline_run.py                    # M1 悬停
python3 tools/offline_run.py --all              # 全部工况
python3 tools/offline_run.py --plot             # 另存 report/figures/offline_*.png
python3 tools/offline_run.py --scenario M2_line_v10 --compare   # 开环 vs 闭环
python3 tools/offline_run.py --sweep-noise      # 释放/B 初值噪声扫描
python3 tools/offline_run.py --scenario M4_high_kf --mc   # M4 蒙特卡洛（估计器对比）
```

## 已有结果（离线）

**M1 悬停释放**：`h_c=2.00m`、`v_p=3.14 m/s`、速度失配 0；PASS，最近 0.12 m。

**M2 带速抛投**（A 平飞 0.5 / 1.0 / 2.0 m/s + 斜向）：

| 工况 | 释放点 `p_r` | 释放速度 | 会合点 `h_c` | 载荷速度 | 仿真 |
|---|---|---|---|---|---|
| v05 | (−2.10, 0, −3) | (0.5,0,0) | 1.96 m | 4.51 m/s | PASS |
| v10 | (−0.30, 0, −3) | (1.0,0,0) | 1.96 m | 4.51 m/s | PASS |
| v20 | (+3.20, 0, −3) | (2.0,0,0) | 1.96 m | 4.51 m/s | PASS |
| crosswind | (+0.60, +1.16, −3) | (1.0,0.6,0) | 1.96 m | 4.51 m/s | PASS |

**M3+ B 的终端 MPC（acados）**：`controller='mpc'` 时 B 用 acados 终端 MPC
（状态 `[p,v]`、控制 `u=a`；跟踪解析会合参考 + 终端代价；`|u|≤a_max` 硬、`|v|≤v_max` 软）。
与 PD（解析参考 + 前馈 + PD）对比：

| 工况（无扰动） | PD | MPC |
|---|---|---|
| M2_line_v10 短下落 | 10/10，捕获距离 0.16m | 10/10，0.27m |
| M3_high 高抛 | 10/10，0.29m | 10/10，0.28m |

**发现**：短下落场景下 MPC 捕获略松。原因不是 MPC 有 bug，而是**解析参考的几何**：
`M2_*` 里 B 有 ~4.2s 但只需净下移 0.54m、且要以 4.5m/s 结束，min-energy 三次多项式
必须让 B **先爬升 ~2.4m 再俯冲**（对偶：B 得“蓄势”才能匹配快速载荷）。PD 带解析前馈
死跟这条参考；MPC 最小化控制量，倾向于抄近路不走那段过冲 → 终点偏离参考。
**改参考的努力（`w_overshoot` / `reference_mode='staged'`）与结论：**

为减少那段"先爬升后俯冲"，实现了两个旋钮：
- `planner.w_overshoot`：代价里惩罚 B 轨迹超出 `[起始,会合]` 高度包络的过冲；
- `planner.reference_mode='staged'`：解析生成"先送到最优待机高度 `h_stage=h_c+v_c²/(2a)`、
  再末端匀加速俯冲"的分段参考（`reference_a_frac` 控制俯冲用多少 a_max）。

| 参考 | 过冲 | 峰值\|a\| | σ=0.15 成功率(PD) |
|---|---|---|---|
| `cubic`（min-energy，默认） | 2.39 m | 4.45/6.0（余量足） | 8/10 |
| `staged`（分段俯冲，用满 a_max） | 1.22 m | 6.00/6.0（无余量） | 6/10 |
| `staged` + `reference_a_frac=0.8` | ~1.6 m | 4.80/6.0 | 更差 |

**结论：过冲不是单纯的缺陷，而是 min-energy 参考"留反馈余量"的副产品。**
min-energy 三次只用了约 74% 的 `a_max`，剩余额度正好被 PD/MPC 用来纠正释放误差；
把它压到"不爬升"就得用满加速度，鲁棒性反而下降。**因此默认保留 `cubic`**，
分段参考作为可选（`reference_mode: staged`）。要真正兼顾，得从**会合几何**入手
（如让 B 的待命点更高、有天然下滑跑道），而不是只改参考。

**M3 闭环重规划**（开环 vs 闭环，每档 12 次）：

- **释放误差**（高抛，下落 ~0.8 s）：`σ=0.20 m` 时 开环 **7/12** → 闭环 **12/12**
  （带 0.05 m 载荷测量噪声仍 12/12）。
- **模型失配**（风 [2,1,0] m/s + 线性阻力 k=2，规划器仍按无阻力预测）：
  开环 **0/10** → 闭环 **10/10**（`replan_dt=0.3s` 即可）。

结论：闭环重规划能显著提升释放误差与模型失配下的捕获成功率。

**M4 鲁棒性：载荷状态估计 + 测量延迟/丢包**（`payload_filter.py`）：

- 测量管线：位置噪声 `pos_sigma`、延迟 `latency_s`、丢包 `dropout`（scenario 单一真值源）。
- 估计器 `estimator.mode`：`none`（用真值，理想上界）| `naive`（最近测量+有限差分+弹道外推）|
  `kf`（卡尔曼滤波：状态 `[p,v]`、重力为已知输入、按测量时间戳 predict→update，天然处理延迟/丢包）。
- 蒙特卡洛：`python3 tools/offline_run.py --scenario M4_high_kf --mc`。

| 测量严重度 | 估计误差 naive | 估计误差 KF | 成功率 naive | 成功率 KF | 成功率 none(真值) |
|---|---|---|---|---|---|
| 轻 (0.03m/0/0) | 0.049 m | **0.037 m** | 27/30 | 30/30 | 30/30 |
| 中 (0.05m/0.1s/20%) | 0.667 m | **0.189 m** | 30/30 | 27/30 | 30/30 |
| 重 (0.10m/0.2s/40%) | 0.305 m | **0.164 m** | 29/30 | 29/30 | 30/30 |

**结论（含一个负结果）**：KF 把估计误差稳定降低 **2–4 倍** ✅，但**端到端成功率没有提升**
（收紧捕获判据到 `r_c=0.15/v_c=0.8`、n=50 时：naive 46/50、KF 43/50，差异在统计噪声内）。
关键旁证：**用真值的 `none` 也只有 46/50** ⇒ **瓶颈不在估计精度**，而在释放误差与 B 的动力学限幅。
所以"更好的估计"在这套几何/判据下不是增益点；KF 的价值要等跟踪/判据成为瓶颈时才体现。

## M6 垂直堆叠投放（A 正上方释放 + B 温和下潜 + 刚性漏斗）

任务形态：A、B 地面水平相距 5m 起飞、各自升到不同高度，B 飞到 A **严格正下方**；
两机水平速度均为 0、投影重合时 A 释放。载荷＝纯垂直自由落体；B 在下方以 `a_dive`
**温和下潜（软着陆）**，随后以 `a_brake` 刹停。

**关键物理**：B 只能往下压（`a_B < g`），纯垂直下落的接触相对速度有下界

```
v_rel = √( 2·(g − a_dive)·gap )
```

gap=1m 时：悬停硬接 4.43 m/s；a_dive=3 → 3.69；a_dive=6 → 2.76（但 B 冲得低、刹车余量只剩 0.35m）。
**a_dive=3 是冲击/余量的折中。**

**捕获判据（比固定 `v_c` 更严谨，来自刚性漏斗物理）**：

```
位置：落到漏斗口平面时 水平偏差 < mouth_radius − object_radius
速度：接触相对速度 ≤ v_retain = √(2·g·depth) / e      (e = 恢复系数)
```

推荐标称（`config` 里已是）：`A=4.5m, B=3.5m, gap=1.0m, a_dive=3, 漏斗口半径0.2m/深0.3m/e=0.6`
→ `v_rel=3.69 ≤ v_retain=4.04`，离线 200/200 捕获（水平偏差 max 0.096m < 口内有效半径 0.15m）。

```bash
python3 tools/stack_run.py            # 单次
python3 tools/stack_run.py --sweep-dive
python3 tools/stack_run.py --sweep-gap
python3 tools/stack_run.py --mc 200
python3 tools/stack_run.py --sweep-wind   # 侧风干扰鲁棒性（见 report/robustness_wind.md）
```

**干扰鲁棒性（侧风，新增）**：载荷加线性阻力 + 常值侧风后会水平漂移。原始的"PD 追尾"
（`v_ref,xy=0`）稳态滞后 `e≈kd·v/kp`，侧风容忍仅 ~1 m/s。加**速度前馈**（`v_ref,xy=v̂`）
→ ~2.5 m/s，再加**预测式对正**（`p_ref = p_meas + lead·v̂·t_rem`）→ **~3.5 m/s（≈3.5×）**；
但相对定位噪声会把边界拉回 ~1.5 m/s（瓶颈转为估计器）。详见 `report/robustness_wind.md`。

> ⚠️ 现有 3D `RendezvousPlanner` **不适合**本场景：它求最小代价时会让 B 爬到 A 正下方 ~0.1m 处，
> 或要求峰值加速度 15–600 m/s² 来匹配末端速度。故 M6 用独立的解析规划器（`stack_drop.py`）。

### M6 SITL（已跑通 ✅）

```bash
source ~/drone_payload_catch/env.sh
bash ~/drone_payload_catch/run_m6_sitl.sh 70      # 默认 A 4.5m / B 3.5m / 水平 5m 外起飞
```

实测结果（`~/payload_catch_m6/launch.log`）：

```
B: CLIMB done → WAIT_A（先垂直爬升，绝不平移，避免斜插进 A 的爬升通道）
B: WAIT_A done (A_alt=4.40, clear=0.90) → TRANSLATE
B: TRANSLATE done → ALIGN
B: ALIGNED rel_xy=0.045m spd_xy=0.032 → release
B: DIVE plan t_c=0.449s v_rel=3.059 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz=0.013m rel_v=1.748m/s
B phase=DONE  （载荷骑在漏斗上跟着 B 悬停 → “接住→带走”）
    全程 min|A−B| ≈ 0.89m（无碰撞）
```

**起飞避碰**：B 必须 `先垂直爬升 → 等 A 爬到悬停高度且垂直速度归零 → 再定高横移到 A 正下方`；
若 B 直接斜插到 A 正下方，斜插路径会穿过 A 的垂直爬升通道→相撞。横移/悬停时全程保证
B 高度 ≤ A 高度 − `min_ab_gap`（默认 0.8m），`b_node` 打印 `min_relA` 作碰撞监测。

**任务收尾（落地）**：捕获后保持 `land_after_catch_s`（默认 6s），A、B **各自先飞到分开的着陆点
`land_xy`（A≈[-4,0]、B≈[5,0]，相距~8.5m，避免落一起），再发 `VEHICLE_CMD_NAV_LAND` 自动降落并停发 offboard**；
PX4 日志会看到两机 `Landing detected → Disarmed by landing`。载荷跟随漏斗一起落到地面。

关键实现：
- **B 用带刚性漏斗的自定义模型** `models/x500_funnel`（`<include merge> model://x500` + 顶部圆锥）；
  先 `gz service create` 成 `x500_funnel_1`，再以 `PX4_GZ_MODEL_NAME` 让 PX4 attach——**不改 PX4 树**。
  启动文件 `launch/catch_stack_launch.py`，一键 `run_m6_sitl.sh`。
- **相对定位（mesh 替身）**：A 广播 `/drone_a/state`（世界系 NED），B 订阅后加**延迟/抖动/丢包/慢变偏置/噪声**，
  再经 **EMA 低通**（`est_lpf_alpha≈0.30`）滤波后才用于对正判断与控制——否则重噪声会把对正门限卡死。
- **载荷闭环跟踪**：DIVE 阶段 B 跟踪**载荷本身**（带测量噪声/延迟），而非 A，闭环纠正释放误差。
- **难度扫描**：`bash tools/sweep_m6_sitl.sh`（10 档）→ `report/m6_sitl_results.md`；
  **蒙特卡洛** `NREP=5 bash tools/mc_m6_sitl.sh` → `report/m6_sitl_mc.md`。关键结论：
  - 标称 **5/5**；重噪声下**不加 EMA 滤波 1/5** vs 加滤波 **4/5**；
  - 释放误差 σ=0.20 时**跟踪载荷 5/5** vs 不跟踪 **1/5**（闭环跟踪决定性）；σ=0.30 跟踪仍 5/5；
  - 释放提前量 0.45s / 落差 1.2m / 下潜 a=5 均稳定捕获并落地。
  - 局限：成功水平误差~0.13m 贴近漏斗有效半径 0.14m（余量小）；重噪声下 `min A-B` 估计不可靠，
    安全层应加上估计不确定度余量。
- **释放时序**：B 飞到 A 正下方、水平速度归零且 A 也稳定后，广播 `/payload/release_at`；
  载荷由 `payload_node` 瞬移到 **A 实际位置下方 0.15m**（避免在 A 机体内生成被弹飞）。
- **捕获判据**：载荷落到漏斗口平面（注意 PX4 `pos_world.z` 比模型绝对高度低 0.24m，需 `px4_z_bias`）、
  水平偏差 < 0.14m、相对速度 ≤ `v_retain`。

> 局限：当前漏斗是**实心圆锥宽口朝上 = 平顶盘**（靠低恢复系数接触面“砸住”），不是空心导向漏斗；
> B 开机即有固有健康告警（`Preflight Fail: Attitude failure (roll)`），不影响任务。

### M6-moving 编队同速投放（已跑通 ✅·加难度）

任务升级：A、B 先到**同一投影点**悬停，然后以**相同的小速度同向巡航**，在**运动中**释放。

```bash
source ~/drone_payload_catch/env.sh
FORMATION_VEL="0.5,0.0,0.0" bash run_m6_gui.sh 70    # 可视化
FORMATION_VEL="0.5,0.0,0.0" bash run_m6_sitl.sh 70   # 无窗口
```

流程（`mode=stack` + `formation_vel≠0`）：
```
CLIMB → WAIT_A → TRANSLATE → ALIGN（同一投影点悬停，rel_xy<0.12）
  → B 发 /formation/start
  → A 以 formation_vel 直线巡航；B 进入 FORMATION（目标=A 投影点，前馈=A 速度）
  → 位置+速度都对正（rel_xy、rel_vxy 均 < align 阈值且稳定）→ A 释放
  → DIVE（跟踪 A，等物块真正下落后下潜）→ 刚性漏斗捕获（物块落在漏斗上被带走）→ 双机分开落地
```

实现要点：
- **载荷速度继承**：`models/payload_attached` 用 gz `DetachableJoint` 把载荷挂在 A 下方随飞，
  分离时物块**继承 A 的速度**（物理正确）；`payload_node` 在 A 正下方生成并挂载（configure 即挂载），
  释放时 s 分离（不再瞬移）。**切不可重复发 attach**（会叠加固定关节 → 过约束 → 物块被甩飞）。
- **几何**（编队模式自动设定）：`A=5.0m, B=3.3m, RELEASE_Z=0.45m`——偏移需大于 A 起落架（~0.23m）避免被弹飞，
  且 A/B 间距拉开以保证下落高度。
- **A**：收到 `/formation/start` 后 `v = formation_vel + kp·(参考点−当前点)`（置参考+前馈）；
  捕获后等待降落期间**原地保持**（不再飞回原点）。
- **B**：`FORMATION` 相位用**已知 `formation_vel` 死推算参考** + 相对小幅校正（`form_kp_rel`），
  **不再用延迟/带噪的 A 速度估计做前馈**（否则高速发散）；释放需位置与相对速度双阈值
  （`rel_vxy` 对已知 `formation_vel`，低噪）且 A 达到 `formation_min_speed_ratio·|v_form|`。
- **及时释放 + 超时中止**（新增）：容忍短暂抖动（`align_reset_tol`）→ 编队 **~3.5s 即捕获**（之前 10–20s）；
  超时（`formation_timeout_s`）未释放 → `/formation/abort` **中止投放**（A 保留载荷、双机安全落地）。
- **支持速度**：`0.5 / 1.0 m/s`（各 2/2 完美：捕获 + 保持 + 双机落地，无 failsafe）；`2.0 m/s` 保留、不再优化。
- **DIVE 重锚**：分离有 ~0.1–0.2s 延迟，B 先**原地悬停**等物块真正下落（`vz>dive_anchor_vz`）
  再重锚下潜；编队模式跟踪 **A**（无释放误差，比跟踪物块估计更稳）。
- **捕获判据**：物块必须在漏斗口平面上下窗口 `±catch_z_tol` 内，避免“物块落地后被误判捕获”与
  “还挂着就误判捕获”。

实测（`FORMATION_VEL=0.5`）：
- **无窗口 SITL 连续 3 次全成功**：`FORMATION aligned rel_xy 0.011–0.067m / rel_vxy 0.061–0.187`
  → `STACK CAPTURED horiz 0.108–0.128m` → 物块停在漏斗上（`z≈-3.6~-3.7`）随 B 飞到落点，A/B 均无 failsafe。
- **GUI 可视化多数成功但偶发失败**：失败均为 A 端 `Failsafe activated`（`Attitude failure (roll)` +
  `time jump detected`）——Gazebo 渲染负载拖慢实时性→PX4 仿真时间跳变→飞控 failsafe，
  **属平台级问题（见 MEMORY），非捕获算法缺陷**。Gazebo GUI 默认**不跟随相机**（固定全场视角）。
- **余量**：捕获水平偏差 0.108–0.128m vs 有效半径 0.14m，余量仅 0.01–0.03m（偏小）；
  物理漏斗盘半径其实是 0.20m，还有提升空间。

## SITL 环境（B 阶段，见 `report/env_bringup.md`）

```bash
source ~/drone_payload_catch/env.sh    # acados + ROS + RMW=fastrtps + PX4 gz 资源路径
```

- 本机 PX4 是 **main**（`cdecd90`），对应 `px4_msgs` 提交 **`ee2e90c`**（已切分支
  `main-cdecd90` 并编译）。
- DDS 链路已验证通（`/fmu/out/vehicle_status_v4` @2Hz）。
- **传感器桥 `Gyro STALE` 未解决**；根因是这台机器的 PX4 检出被本地大改、无法干净重建
  （详见报告）。因此 M1 的 ROS 接入改用 `real_hardware_launch` 同级架构，等环境修复后再联调。

## 配置（单一真值源）

`config/catch_scenarios.yaml` 驱动全部几何 / 参数 / 工况：

- `defaults`：重力、控制频率、载荷（质量/阻力/风）、B 的 `max_speed/max_accel`、
  捕获阈值 `r_c/v_c`、规划器搜索范围与权重。
- `layouts`：A 初始位置+速度 `a_init/a_vel`、B 待命点 `b_standby`（NED）。
- `scenarios`：`M1_*`（悬停）、`M2_*`（带速抛投）、`M3_*`（闭环，可带 `closed_loop: true`）；
  scenario 可覆盖 `payload` / `wind` / `drone_b` / `planner` / `capture` / `a_init` / `a_vel`。
- `thresholds`：离线报告判定阈值。
- `capture.funnel`（M6）：刚性漏斗 `mouth_radius/depth/restitution/mount_height/object_radius`。
- `stack`（M6）：B 下潜加速度 `a_dive`、刹车 `a_brake`。

## 目录

| 路径 | 说明 |
|---|---|
| `payload_catch/payload_model.py` | 载荷抛体模型（无阻力解析；可选 linear/quadratic 阻力 + 风） |
| `payload_catch/rendezvous.py` | 协调求解 `(t_r,τ_c)` + 三次多项式会合参考 + 软终端速度 + 闭环 `solve_inflight` + 协同 `solve_cooperative` |
| `payload_catch/sim_core.py` | 离线闭环仿真（A 恒速飞行 + B 控制 + 捕获判定 + 闭环重规划） |
| `payload_catch/mpc_terminal.py` | B 的 acados 终端（会合）MPC |
| `payload_catch/payload_filter.py` | 载荷状态估计（卡尔曼滤波 / 朴素对照） |
| `payload_catch/stack_drop.py` | M6 垂直堆叠投放（解析规划 + 漏斗保持判据 + 离线仿真） |
| `models/x500_funnel/` | M6：x500 + 顶部刚性捕获圆锥（PX4_GZ_MODEL_NAME 附着） |
| `models/x500_funnel_big/` | M6 末端能力：同构但口半径 0.30m（`FUNNEL_MOUTH=0.30` 启用） |
| `models/payload_attached/` | M6-moving：带 `DetachableJoint` 的载荷（挂 A 随飞、分离继承速度） |
| `launch/catch_stack_launch.py` / `run_m6_sitl.sh` | M6 SITL 启动 / 一键脚本 |
| `tools/stack_run.py` | M6 体检 CLI（`--sweep-dive` / `--sweep-gap` / `--sweep-wind` / `--mc` / `--lead`） |
| `tools/offline_run.py` | 体检报告 CLI（`--plot` / `--sweep-noise` / `--compare`） |
| `config/catch_scenarios.yaml` | 单一真值源 |
| `env.sh` | 环境变量（acados/ROS/RMW/PX4 SITL） |
| `report/env_bringup.md` | B 阶段环境打通记录（含 PX4 检出问题与回滚清单） |
| `report/survey_and_sim_report.md` | **方向综述 + 完整仿真报告**（推荐新读者先看） |
| `report/mission_overview.md` | **方向总述：被控对象 · 控制算法 · 仿真图表**（含架构/物理/轨迹图） |
| `report/handoff_exploration.md` | **交接场景深度探索**：静态/动态/机械臂/增加速度的公式推导与边界 |
| `report/robustness_wind.md` | **侧风干扰鲁棒性优化**：速度前馈 + 预测式对正，侧风容忍 1→3.5 m/s |
| `report/m6_robustness_opt.md` | **鲁棒性优化 II**：KF 估计 / 自适应下潜 / 漏斗几何扫掠（几何是主杠杆） |
| `report/end_effector_bigfunnel.md` | **末端能力**：大漏斗 0.20→0.30m，离线余量×14 + SITL `STACK CAPTURED` |
| `report/robust_geometry_and_retention.md` | **鲁棒几何 + 末端机构**：风下免下潜规则 `gap≤v_retain²/2g`；空心锥/主动保持对比 |
| `report/hover_first_control.md` | **悬停优先控制**：`minimal_dive` 落到 b_node（`auto_min_dive`），SITL `a_dive=0` 捕获 |
| `report/drag_rejection.md` | **抗阻力/风扰**：二次阻力 + 阵风；**A 端迎风预补偿**（w=8: 0→40/40） |
| `report/hollow_funnel_cup.md` | **空心导向锥杯**：建模+SITL（含负结果：敞口杯倾斜不如高摩擦平盘） |
| `report/active_retention.md` | **主动保持（锁扣）**：离线（下击暴流100/100）+ SITL 捕获→锁定→携带→落地 |
| `report/coordination.md` | **双机协调**：现状/缺口/改进清单（握手/意图/时钟/安全）；含负结果 |
| `report/coordination_handshake.md` | **协同释放握手**：B 报就绪→A 作释放权威→ack→下潜；SITL 验证 |
| `report/coordination_validation.md` | **协同协议 SITL 验证**：不变量（就绪→释放→ack/单次）+ 安全间隔 + 无 failsafe |
| `report/optimization_backlog.md` | **后续可优化项总表**：按层整理 + 优先级 + 已证负结果 + Top-3 |
| `report/opt_round2.md` | **优化第二轮**：分级安全状态机(HOLD/PULLBACK/LAND/KILL) + 加速度前馈/速率限幅 + σ共享 + 释放提交/取消窗口 + 3D keep-out + 在线自适应下潜 |
| `report/opt_round3.md` | **优化第三轮（安全硬化）**：安全指令绕过速率限幅 + 围栏软限幅 + 安全状态进日志 |
| `report/opt_round4.md` | **优化第四轮（传感器约束）**：用 PX4 EKF 已有 `eph/epv` 作 σ + 健康/一致性看门狗 + 估计器限值 |
| `report/opt_round5.md` | **优化第五轮（姿态约束）**：IMU 倾角/角速率 → 安全滤波（水平指令衰减 + 加速度限额），对标姿态 failsafe 边界 |
| `report/research_roadmap.md` | **研究路线图（协调方向）**：目标/UAV协调研究地图 + 核心贡献 C1–C5 + 现状差距 + 理论工具 + 分阶段实验 + 代码映射 + 最小可发表单元 |
| `report/paper_outline.md` | **论文骨架（协调方向）**：题目/摘要/贡献 C1–C5/问题形式化/方法/实验协议/相关工作定位/真机需求/时间线/待定决策 |
| `report/coordination_probability.md` | **C1 概率证书（v2 精确 Rice）**：二维脱靶模型 + 精确 `P(capture)≥1−ε` 证书 + exact/Chernoff/heuristic 对比 + EMA 标定（“标准漏斗 rel_σ≥0.10 无法认证”）|
| `report/coordination_benchmark.md` | **协同交接基准（阶段 0）**：`tools/bench_coord.py` 生成（coord_mode×intent×σ×delay + 置信区间） |
| `report/planning_control_opt.md` | **规划/协调 + 控制优化**：ZEM 终端导引 + 释放前落点余量闸 |
| `report/safety_control_review.md` | **保护控制审查**：已有（限幅/keep-out/释放闸）vs 缺口（geofence/看门狗/abort/避碰） |
| `report/safety_supervisor.md` | **安全监督 + 飞行终止(kill)**：外部 `/safety/kill_a|b` + 异常自动 kill；SITL 验证 |
| `report/m6_moving_speed.md` | **M6-moving 加速度**：编队控制优化 + 及时释放/超时中止；**支持 0.5/1.0 m/s（各 2/2 完美）**，2.0 保留 |
| `tools/validate_coord.py` | 协同协议 SITL 验证器（跑多组配置 + 不变量检查） |
| `tools/bench_coord.py` | **协同交接基准（阶段 0）**：coord_mode×intent×σ×delay 网格 + Wilson CI + 报告 |
| `tools/coord_prob.py` | **C1 概率证书**：二维脱靶模型 + 证书 k(ε) + 策略/延迟扫描（离线、秒级） |
| `tools/drag_reject.py` | 阻力/风扰 × 估计器对比 |
| `tools/gen_funnel_cup.py` | 生成空心导向锥杯模型（x500_funnel_cup / funnel_cup / funnel_flat） |
| `models/x500_funnel_cup/` | M6 末端：x500 + 空心导向锥杯（`FUNNEL_TYPE=cup` 启用） |
| `models/payload_lock/` | M6 主动保持：带 B 侧 `DetachableJoint` 的载荷（`PAYLOAD_LOCK=1`，捕获时就地重生成并锁到 B 漏斗） |
| `tools/geom_opt.py` | 鲁棒几何网格搜索（成功率 + p10 余量） |
| `tools/funnel_model.py` | 末端机构解析对比（平顶盘/空心锥/主动保持） |
| `tools/handoff_explore.py` | 交接公式推导的数值验证 CLI |
| `tools/make_report_figures.py` | 一键生成总述文档用的仿真图表（PNG） |
| `payload_catch/px4_iface.py` | PX4 无人机接口基类（话题/QoS/ARM+OFFBOARD/setpoint） |
| `payload_catch/{a_node,b_node,payload_node}.py` | M1 ROS 节点：A 悬停 / B 会合 / 载荷源 |
| `launch/catch_launch.py` | M1 SITL 启动 |
| `run_m1_sitl.sh` | M1 一键 SITL（gz+2×PX4+agent+节点） |

## 路线图

- [x] **M0** 项目骨架 + 载荷模型 + 会合规划 + 离线闭环
- [x] **M2** A 带速飞行抛投（恒速直线，含斜向）
- [x] **M3** 闭环重规划（`solve_inflight` + 载荷状态噪声），开环 vs 闭环对比
- [x] **M3+** B 的终端 MPC（acados），与 PD 对照
- [x] **B** 环境打通：已用 PX4-1.16 解决（见 `report/env_bringup.md`、`MEMORY.md` §2）
- [x] **M1** ROS 2 节点：载荷状态源 / 规划器 / B 控制器 / 捕获监控 / A 悬停释放（SITL 跑通）
- [x] **M4** 更完整鲁棒性（延迟、丢包、估计滤波）+ 指标统计
- [x] **M6** 垂直堆叠投放（离线 200/200；SITL `STACK CAPTURED` + 双机分开落地）
- [x] **M6 鲁棒/末端** 侧风鲁棒、KF/增广风、**大漏斗**、**空心杯**、**主动保持锁扣**、鲁棒几何优化
- [x] **协同/安全** 释放握手 + 意图（预测落点）+ 时钟同步；不确定度 keep-out + **飞行终止(kill)** + 超时中止
- [x] **M6-moving** 编队同速投放：**支持速度 0.5/1.0 m/s（各 2/2 完美）**；及时释放；2.0 m/s 保留（不再优化）
- [x] **优化 2–5** 分级安全状态机 + 加速度前馈 + σ 共享 + 释放提交/取消 + 3D keep-out + 自适应下潜
  + 传感器/估计器约束（EKF σ / 健康看门狗 / 限值）+ 姿态/角速率安全滤波（`report/opt_round2..5.md`）
- [x] **研究·阶段 0** 协同交接基准实验台 `tools/bench_coord.py` → `report/coordination_benchmark.md`
- [~] **研究·协调方向** 路线图 C1–C5（不确定性释放保证 / 通信鲁棒协议 / state-vs-intent / 联合机动 / handover-CBF）
  → `report/research_roadmap.md`；**待推进**（C1 概率证书 为先）
- [~] **M5** 真空心漏斗（现为 primitive 杯 / 实心盘）+ 真夹爪/磁吸 + 真机化

## 开发约定

- 核心算法层（`payload_model` / `rendezvous` / `sim_core`）**不引入 ROS 依赖**，
  保证可离线验证与移植。
- 改 `config/catch_scenarios.yaml` 后离线直接生效；ROS launch 读安装副本，需 `colcon build`。
- 所有几何/阈值只写在 yaml，不在代码里硬编码。
