# 空投—空中捕获：方向综述与仿真报告

> 文档性质：一篇面向「无人机 A 空投重物、无人机 B 空中接住」方向的**技术综述**，
> 叠加 **本项目（drone_payload_catch）截至目前的完整仿真报告**。
> 生成时间：基于仓库当前状态（M6 已跑通 SITL；载荷模型 0.06 m / 0.3 kg 立方体；漏斗口半径 0.20 m）。
> 读者：项目接手者、评审、或想快速进入该方向的研究者。

---

## 目录

1. [方向综述](#1-方向综述)
   - 1.1 [问题定义与场景谱系](#11-问题定义与场景谱系)
   - 1.2 [与相邻问题的关系](#12-与相邻问题的关系)
   - 1.3 [技术挑战分解](#13-技术挑战分解)
   - 1.4 [国内外研究现状](#14-国内外研究现状)
   - 1.5 [方法与工具谱系](#15-方法与工具谱系)
   - 1.6 [开放问题与趋势](#16-开放问题与趋势)
2. [本项目方法](#2-本项目方法)
3. [仿真报告](#3-仿真报告)
4. [关键发现与负结果](#4-关键发现与负结果)
5. [局限与下一步](#5-局限与下一步)
6. [参考文献（代表性）](#6-参考文献代表性)

---

## 1. 方向综述

### 1.1 问题定义与场景谱系

「空投—空中捕获」研究的是**两个或多个空中平台之间的物体转移**：一个平台（投送者，本文记 A）
在飞行中释放载荷，另一个平台（接收者，本文记 B）在载荷**尚未落地之前**于空中完成捕获与保持。

按几何与动力学可分成一条谱系：

| 形态 | 投送方 | 接收方 | 相对运动 | 典型难点 |
|---|---|---|---|---|
| **垂直堆叠投放**（本项目 M6） | 悬停/低速 | 悬停/低速 | 近共线、纯垂直 | 接触相对速度下界、末端机构 |
| **水平/斜向会合**（本项目 M1–M4） | 恒速平飞 | 机动到位 | 空间会合点 | 会合规划、终端速度匹配 |
| **空射回收**（Gremlins / SideArm 类） | 母机释放/回收 | 母机或回收装置 | 高速、大惯量 | 相对导航、气动干扰、捕获机构 |
| **精确空投**（JPADS / 翼伞投送） | 运输机高空投放 | 地面目标区 | 单向下落 | 落点精度、风场估计 |
| **空中加油 / 交会对接** | 加油机 / 目标星 | 受油机 / 追踪星 | 近场编队 | 相对状态估计、精细控制 |
| **物体抛接（ball catching / juggling）** | 人/机械臂/无人机 | 无人机 | 快速非合作目标 | 状态预测、极短窗口、动态抓取 |

本方向的核心科学问题是：**在存在释放误差、模型失配、测量噪声/延迟、以及双方动力学限幅的条件下，
求解一个时空会合问题，使接收方在满足安全约束的前提下，以尽量小的相对速度到达载荷位置并完成保持。**

### 1.2 与相邻问题的关系

- **空中操纵（Aerial Manipulation）**：捕获机构、机械臂/夹爪设计、接触力控制等可直接复用。
- **无人机协同运输（Cooperative Transport）**：捕获后的"带走"阶段与吊挂/刚体负载运输同源。
- **空间交会与对接（Rendezvous & Docking）**：CW 方程下的终端制导、V-bar/R-bar 逼近、走廊约束，
  和本项目"会合点 + 速度匹配"在数学上高度相似（都是双积分器 + 终端位置/速度约束）。
- **导弹拦截/比例导引（Proportional Navigation）**：非合作快速目标的预测拦截与终端导引律。
- **机器人动态抓取/接球（Dynamic Grasping / Catching）**：从 1980 年代的 "robot catching" 到
  现代高速视觉 + 机械臂，解决"预测—规划—在极短窗口内到位"的同一类问题。

### 1.3 技术挑战分解

把端到端任务拆成六个耦合子问题：

**(1) 会合规划 / 制导（Rendezvous planning & guidance）**
- 决策变量：**释放时刻** `t_r` 与**捕获时刻/下落时长** `τ_c`（本项目联合搜索 `(t_r, τ_c)`）。
- 目标：在可行域内最小化"时间 + 机动代价 + 终端速度失配"。
- 约束：双方速度/加速度限幅、离地余量、捕获高度区间、安全间距。
- 数学本质：带终端位置/速度约束的**双积分器最优控制**，标称解是 min-energy 多项式轨迹。

**(2) 相对导航与状态估计（Relative navigation & estimation）**
- 需要载荷实时状态（位置/速度）与 A–B 相对位置。
- 现实手段：视觉/事件相机、UWB/超宽带、GNSS-RTK、动捕、机间链路（mesh）、雷达。
- 难点：**延迟、丢包、遮挡、非合作目标的视觉特征弱**；估计精度与不确定性量化。

**(3) 捕获控制（Capture control / aggressive maneuvering）**
- 接收方常需**高动态机动**（俯冲、横向急停、终端速度匹配），逼近执行器与姿态控制极限。
- 方法：解析前馈 + PD、LQR、非线性/终端 MPC、微分平坦、几何控制。
- 关键张力：**跟踪精度 vs. 反馈余量**（把参考做到"极限"会牺牲鲁棒性）。

**(4) 末端捕获机构（End-effector / capture mechanism）**
- 软网 / 网枪、机械夹爪、刚性漏斗/兜、磁吸、刺穿/钉住、静电吸附、perching 爪。
- 需要处理**接触瞬间的冲击**：能量耗散（低恢复系数、缓冲）、约束（漏斗口/夹持深度）、
  以及"捕获后保持"（锁定、闭爪、磁力）。
- 与接触力控制、柔性/刚性耦合动力学相关。

**(5) 安全与鲁棒性（Safety & robustness）**
- 双机**避碰**（尤其起飞/接近/释放通道交叉）、失效保护（failsafe）、丢失目标后的应急处理、
  估计不确定度余量、地理围栏。
- 难点：噪声下"估计出来的最小间距"不可靠，安全层需带不确定性裕度。

**(6) 系统集成与真机化（System integration）**
- 传感器/执行器/通信时延、飞控与机载算力、实时性与认证。

### 1.4 国内外研究现状

> 下面按子方向给出代表性脉络，用于定位本项目（引用见 §6）。

**A. 空中操纵与抓取**
- 空中操纵综述给出了"机械臂/夹爪安装在旋翼平台上"的分类与挑战（Ruggiero 2018；Ollero 2022）。
- 早期工作把 2–6 DoF 机械臂装上四旋翼完成抓取与放置（Kim 2013；Thomas 2013/2014 的鸟爪式抓取/停栖）。
- 动态抓取/停栖：受鸟类启发的高速闭合爪与冲击吸收（Roderick 2021；固定翼 post-stall perching, Moore 2014）。
- 这类工作提供了**末端机构的物理模型与接触动力学**，是本项目 M6 漏斗机构的直接参照。

**B. 空中物体抛接（Ball catching / juggling）**
- ETH 团队系统研究过四旋翼**抛球与接球**、球杂耍（Ritz/Müller/D'Andrea 等），
  强调状态预测、极短时间窗口内的轨迹规划与高带宽控制。
- 与本项目 M1–M4"接住下落载荷"同属"预测—规划—到位"范式，但目标更慢、更可控。

**C. 空中回收与发射回收（Aerial recovery）**
- DARPA **Gremlins** 项目、Aurora 的 **SideArm** 回收臂，研究"空中回收无人机"。
- 商业/航天案例：**火箭整流罩/载荷舱空中回收**（SpaceX 用带网船只、Rocket Lab 尝试直升机勾取）；
  **样品返回舱空中回收**（Genesis 2004、Stardust 2006 用直升机钩住降落伞）；
  冷战时期 **Corona/Discoverer 胶片舱**由飞机空中勾取。这些是"空投—空中捕获"在宏观尺度上的经典范例。
- 这类系统的接收方多为**大型有人机/直升机**，捕获靠网/钩，强调**相对导航与接近走廊**。

**D. 精确空投（Precision airdrop）**
- 翼伞（parafoil）精确空投系统 **JPADS** 用 GNSS/INS 制导把货物送到地面落点，代表"投送方自主"路线；
- Zipline 等用固定翼无人机 + 降落伞/纸板减速器实现**包裹定点空投**，代表"末端减速与落点控制"。
- 与本项目的区别：它们是"投送—落地"，本项目是"投送—**空中**接收"。

**E. 空中加油 / 空间交会对接（近场编队）**
- 自主空中加油（AAR，NASA DROID、F/A-18 AAR 等）解决**锥套/受油杆的视觉跟踪与近场编队**；
- 空间交会对接（CW 方程终端制导、V-bar/R-bar 逼近）是**双积分器 + 终端约束**的成熟理论。
- 两者为本项目的会合规划与相对导航提供了可直接迁移的方法论（预测、走廊、速度匹配）。

**F. 制导与控制方法**
- 终端导引：比例导引（PN）及其变体（拦截机动目标）；
- 轨迹优化：min-snap/min-energy 多项式（Mellinger 2011；Richter 2013）；
- 最优控制：凸优化、微分平坦、LQR-Trees；
- 在线优化：**acados/ACADO** 等嵌入式 MPC 框架（Verschueren 2022）已能机载实时求解。

**G. 估计与感知**
- 卡尔曼滤波/EKF/UKF 做目标状态估计与延迟补偿；
- 高速视觉、事件相机用于快速下落/抛掷物体的位姿跟踪；
- 相对导航（UWB/VIO/动捕）用于机间定位。

**华语圈与国内工作**：高校（如北航、上交、浙大、国防科大等）在无人机吊挂运输、空中抓取、
精确空投、集群回收方向有持续研究；工业界（大疆、美团/顺丰物流无人机等）偏重
**投送与落地精度**，空中接收相对少见。整体上，"投送—空中接收"仍属**较空白、偏系统集成**的方向。

### 1.5 方法与工具谱系

| 层次 | 常见方法 | 本项目选择 |
|---|---|---|
| 会合规划 | 网格/采样搜索；凸优化；多项式轨迹；MPC | 解析 min-energy 三次 + `(t_r,τ_c)` 网格搜索 |
| 接收控制 | PID + 前馈；LQR；终端 MPC；几何控制 | PD（解析前馈）+ acados 终端 MPC（对照） |
| 状态估计 | 真值；有限差分+外推；KF/EKF | naive 估计 + KF（对照） |
| 相对导航 | 视觉/UWB/RTK/动捕 | mesh 替身（A 广播真值 + 噪声/延迟/丢包） |
| 末端机构 | 夹爪/网/漏斗/磁吸 | x500 + 顶部刚性圆锥（M6） |
| 仿真 | MATLAB/Simulink；Gazebo；Flightmare；Isaac | PX4 SITL + Gazebo Harmonic + ROS 2 + acados |

### 1.6 开放问题与趋势

1. **非合作/无标记目标**的实时 6-DoF 估计（视觉/事件相机），替代"真值+噪声"的仿真假设。
2. **高动态捕获的鲁棒性**：把可靠速度边界从 ~2 m/s 推向 ~5 m/s（飞控/EKF 姿态鲁棒性、末端平滑）。
3. **真正的空心导向漏斗 + 保持机构**：从"砸住"升级为"导向—夹持—带走"。
4. **软接触与冲击吸收**：柔性网/缓冲/低恢复系数材料的动力学建模与控制。
5. **不确定性感知的安全层**：用估计协方差设置避碰与捕获裕度。
6. **多机协同捕投**（多接收方/多投送方）、**失败后的应急回收**、**真机与适航**。
7. **学习 + 优化的融合**：数据驱动的残差动力学、学习型末端抓取，叠加模型预测控制保证安全。

---

## 2. 本项目方法

### 2.1 任务与硬约定

- **任务**：A 携载重物飞行，在算法求出的**释放时刻/会合点**抛投；B 实时机动到空间会合点接住。
  要求**位置必须到达（硬）**、**相对速度尽量小（软）**。
- **坐标**：世界系 **NED**（x=北, y=东, z=下），高度 = `-z`。Gazebo ENU→NED：`NED=[ENU_y, ENU_x, -ENU_z]`。
- **单一真值源**：所有几何/参数/工况在 `config/catch_scenarios.yaml`，代码不硬编码。
- **核心算法层不依赖 ROS**（`payload_model` / `rendezvous` / `sim_core` / `mpc_terminal` /
  `payload_filter` / `stack_drop`），可离线一条命令验证。

### 2.2 算法层

**(a) 载荷抛体模型**（`payload_model.py`）
```
p_p(τ) = p_r + v_r·τ + ½g·τ²,   v_p(τ) = v_r + g·τ      (τ = t − t_r)
```
无阻力解析；可选 linear/quadratic 阻力与风（鲁棒性输入）。

**(b) 会合协调求解**（`rendezvous.py`）
- A 第一版为恒速直线 `p_A(t)=a_init + a_vel·t`（`a_vel=0` 即悬停）。
- 在 `(t_r, τ_c)` 网格上最小化
  ```
  J = w_time·t_r + w_accel·(峰值加速度 / a_max) + w_vel·|Δv|²  (+ w_overshoot·过冲)
  ```
  约束：`|v|≤v_max`、`|a|≤a_max`、离地余量、捕获高度区间。
- B 的会合轨迹：双积分器、两端位置/速度、`min∫|a|²dt` 的**解析三次多项式**（每轴）；
  终端速度不可行时退为**软终端速度**解析解。

**(c) 闭环重规划**（`solve_inflight`）
- 载荷离手后每 `replan_dt` 用当前观测载荷状态重解会合，抵抗释放误差与模型失配。

**(d) 捕获判定**（M1–M4）
```
|p_B − p_p| < r_c   且   |v_B − v_p| < v_c
```
用载荷真值判定；测量噪声只影响 B 的规划。

**(e) B 的终端 MPC**（`mpc_terminal.py`，acados）
- 状态 `[p,v]`、控制 `u=a`；跟踪解析会合参考 + 终端代价；`|u|≤a_max` 硬、`|v|≤v_max` 软。

**(f) 载荷状态估计**（`payload_filter.py`）
- `none`（真值上界）| `naive`（最近测量 + 有限差分 + 弹道外推）| `kf`（状态 `[p,v]`、重力为已知输入、
  按测量时间戳 predict→update，天然处理延迟/丢包）。

### 2.3 M6 垂直堆叠投放（`stack_drop.py`，独立解析规划器）

- 场景：A 悬停于 B **严格正上方** `gap` 米，两者水平速度为零、投影重合时释放；
  载荷纯垂直自由落体；B 以 `a_dive` **温和下潜**，接触后以 `a_brake` 刹停。
- **关键物理**：B 只能向下压（`a_B < g`），纯垂直下落的接触相对速度有下界
  ```
  v_rel = √( 2·(g − a_dive)·gap )
  ```
  gap=1 m：悬停硬接 4.43 m/s；`a_dive=3` → 3.69；`a_dive=6` → 2.76（但刹车余量只剩 0.35 m）。
  **`a_dive=3` 是冲击/余量的折中。**
- **捕获判据（来自刚性漏斗物理，比固定 `v_c` 更严谨）**：
  ```
  位置：落到漏斗口平面时，水平偏差 < mouth_radius − object_radius
  速度：接触相对速度 ≤ v_retain = √(2·g·depth) / e     （e = 恢复系数）
  ```
  推荐标称：`A=4.5 m, B=3.5 m, gap=1.0 m, a_dive=3, 漏斗口 0.20 m/深 0.30 m/e=0.6`
  → `v_rel=3.69 ≤ v_retain=4.04`，口内有效半径 `0.20−0.05=0.15 m`。
- **为什么不用 3D `RendezvousPlanner`**：它求最小代价时会让 B 爬到 A 正下方 ~0.1 m 处，
  或要求峰值加速度 15–600 m/s² 来匹配末端速度，均非"B 在下等、载荷掉下来"。

### 2.4 SITL 集成

- **平台**：PX4-1.16（`~/drone_package_20260908/PX4-Autopilot-1.16`）+ px4_msgs v1.16.2 +
  Gazebo Harmonic 8.13 + ROS 2 Jazzy + acados，`RMW=rmw_fastrtps_cpp`。
- **双机**：A=`x500`（PX4 spawn）；B=`x500_funnel`（自定义模型：`<include merge>` 复用标准 x500，
  顶部加刚性圆锥，先 `gz service create` 再由 PX4 attach——**不改 PX4 树**）。
- **载荷**：Gazebo 真实物理体（`models/payload`，当前 **0.06 m / 0.3 kg 橙色立方体**，
  `OdometryPublisher` 发 `/payload/odom`）；启动时生成在远处停车位，释放时瞬移，避免服务延迟。
- **相对定位（mesh 替身）**：A 广播 `/drone_a/state`（世界系 NED），B 订阅后加
  延迟/抖动/丢包/慢变偏置/噪声，再经 **EMA 低通**（`est_lpf_alpha≈0.30`）滤波后使用。
- **M6 状态机**：`CLIMB`（垂直爬升）→ `WAIT_A`（等 A 到位且 `clear≥min_ab_gap`）→
  `TRANSLATE`（定高横移）→ `ALIGN`（对正）→ 释放 → `DIVE`（跟踪载荷下潜）→
  `DONE`（锁定悬停点）→ `LAND`（A/B 各飞分开落点 `land_xy` 后自动降落）。
- **M6-moving 编队同速投放（新增）**：`ALIGN` 后不立即释放，而是由 B 发 `/formation/start`，
  两机以相同小速度 `formation_vel` 同向巡航：A 用"位置参考 + 速度前馈"直线飞行；B 进入 `FORMATION`
  相位，目标为 A 投影点、前馈为 A 速度，位置与相对速度双阈值满足并保持后才释放。载荷用
  Gazebo `DetachableJoint` 挂在 A 下方随飞，分离时**继承 A 的速度**（物理正确）。
- **避碰**：B 必须先垂直爬升再横移（否则斜插穿 A 的爬升通道相撞）；
  全程保证 B 高度 ≤ A − `min_ab_gap`（默认 0.8 m），并打印 `min_relA` 监测。

---

## 3. 仿真报告

### 3.1 实验设置与环境

| 项 | 说明 |
|---|---|
| 操作系统 | WSL2 Ubuntu-24.04，Gazebo GUI 经 WSLg 显示 |
| 飞控 | PX4-1.16 SITL，双实例（`-i 0` / `-i 1`） |
| 中间件 | ROS 2 Jazzy，`rmw_fastrtps_cpp`，MicroXRCEAgent UDP 8888 |
| 优化 | acados（终端 MPC），指纹缓存预热 |
| 载荷 | Gazebo 刚体；当前 0.06 m 立方体 / 0.3 kg |
| 复制命令 | 见 `SIM_COMMANDS.md`（离线 / Gazebo GUI / 扫描 / 蒙特卡洛） |

### 3.2 离线（纯 Python）结果

**单元自测**（`python3 -m payload_catch.*`）
```
payload_model: 解析/积分 err=2.44e-13, fall_time(1.5m)=0.5530s
rendezvous:    hover feasible t_r=2.35 t_c=2.670 h_c=2.00 v_p=[0 0 3.14] dv=0.000
sim_core:      M1_basic miss=0.1182m / M2_line_v10 miss=0.1562m
```

**全工况体检**（`python3 tools/offline_run.py --all`，13 个工况）

| 工况 | 最近距离 | 备注 |
|---|---|---|
| M1_basic / M1_wind | 0.118 m | 悬停释放 |
| M2_line_v05 / v10 / v20 | 0.160 / 0.156 / 0.150 m | 带速抛投 |
| M2_crosswind | 0.151 m | 斜向 |
| M3_high | 0.283 m | 高抛 |
| M3_wind_drag | 0.117 m | 闭环重规划 |
| M3p_line_mpc / M3p_high_mpc | 0.272 / 0.283 m | acados MPC |
| M4_line_naive / kf / high_kf | 0.149 / 0.149 / 0.274 m | 估计滤波 |
| M6_stack_drop | FAIL（**预期**） | 改用 `stack_run.py` |

**M3 闭环重规划（开环 vs 闭环）**
- 释放误差（高抛，下落 ~0.8 s）：`σ=0.20 m` 时开环 **7/12** → 闭环 **12/12**（带 0.05 m 测量噪声仍 12/12）。
- 模型失配（风 `[2,1,0]` m/s + 线性阻力 k=2，规划器按无阻力预测）：开环 **0/10** → 闭环 **10/10**（`replan_dt=0.3s` 即可）。
- **结论：闭环重规划显著提升释放误差与模型失配下的成功率。**

**M3+ B 的终端 MPC（acados）vs PD**

| 工况（无扰动） | PD | MPC |
|---|---|---|
| M2_line_v10 短下落 | 10/10，0.16 m | 10/10，0.27 m |
| M3_high 高抛 | 10/10，0.29 m | 10/10，0.28 m |

短下落时 MPC 捕获略松。原因是**解析参考的几何**：min-energy 三次在时间充裕、终端速度大时
会让 B **先爬升 ~2.4 m 再俯冲**；PD 死跟该参考，MPC 最小化控制量会"抄近路"→ 终点偏离。

**M4 载荷状态估计（KF vs naive）**（`--mc`，每档 30 次）

| 测量严重度 (pos σ/latency/dropout) | 估计误差 naive | 估计误差 KF | 成功率 naive | 成功率 KF | 成功率 none(真值) |
|---|---|---|---|---|---|
| 轻 (0.03 m / 0 / 0) | 0.049 m | **0.037 m** | 27/30 | 30/30 | 30/30 |
| 中 (0.05 m / 0.1 s / 20%) | 0.667 m | **0.189 m** | 30/30 | 27/30 | 30/30 |
| 重 (0.10 m / 0.2 s / 40%) | 0.305 m | **0.164 m** | 29/30 | 29/30 | 30/30 |

**结论**：KF 把估计误差稳定降低 **2–4 倍**，但**端到端成功率没有提升**
（收紧判据 `r_c=0.15/v_c=0.8`、n=50：naive 46/50、KF 43/50，差异在统计噪声内）。
关键旁证：**用真值的 `none` 也只有 46/50** ⇒ **瓶颈不在估计精度**，而在释放误差与 B 的动力学限幅。

**M6 垂直堆叠投放**（`python3 tools/stack_run.py --mc 200`）
```
成功率 = 200/200 (100%)
最近距离:      mean=0.0987 m, max=0.1328 m
捕获相对速度:  mean=3.556 m/s, max=3.560
捕获时水平偏差: mean=0.0331 m, max=0.0960 m   (口内有效半径=0.15 m)
```
`a_dive` 扫描与 `gap` 扫描见 README 表格；标称 `a_dive=3` 为冲击/余量折中。

### 3.3 SITL（Gazebo + PX4）结果

**M5 难度扫描**（`report/m5_sitl_results.md`，B 用 acados MPC）

| 档 | B 横向偏移 | A 高度 | 载荷到达速度 | 捕获 | 最近距离/相对速度 | B failsafe |
|---|---|---|---|---|---|---|
| L0a/L0b | 0.2 m | 3.0 m | 2.06 m/s | ✅✅ | 0.27/0.40 m | 0 |
| L1 | 0.5 m | 3.0 m | 2.06 m/s | ✅ | 0.498 m / 1.37 m/s | 0 |
| L2 | 0.8 m | 3.0 m | 2.06 m/s | ✅ | 0.246 m / 2.10 m/s | 0 |
| L3/L4 | 0.2/0.5 m | 4.0 m | 4.90 m/s | ❌❌ | — | **2** |

- 横向偏移不是瓶颈（0.2→0.8 m 全过）；**真瓶颈是载荷下落速度**：`v_p≈4.9 m/s` 触发 B 的
  `Attitude failure (roll)` / `Compass` + `Battery` failsafe，全部失败。

**M6 SITL 难度扫描**（`report/m6_sitl_results.md`，每档 1 次）

| 试验 | 加压项 | 捕获 | 水平误差 | 接触速度 | min‖A−B‖ | 两机落地 |
|---|---|---|---|---|---|---|
| T0_nominal | 标称 | ✅ | 0.041 m | 2.00 m/s | 0.980 m | ✅ |
| T1/T2 | 释放误差 σ=0.10（跟踪/不跟踪） | ✅✅ | 0.036/0.034 | ~2.0 | ~0.97 | ✅ |
| T3 | 重相对定位噪声 + EMA 滤波 | ✅ | 0.017 | 2.18 | 0.817 | ✅ |
| T3n | 同上，**不滤波** | ❌ | — | — | 0.481* | — |
| T4 | 释放提前量 0.45 s | ✅ | 0.043 | 2.01 | 0.970 | ✅ |
| T5 | 落差 gap=1.2 m | ✅ | 0.048 | 0.805 | 1.175 | ✅ |
| T6 | B 下潜 a=5 | ✅ | 0.016 | 1.315 | 0.971 | ✅ |
| T7/T8 | 释放误差 σ=0.20（跟踪/不跟踪） | ✅✅ | **0.009**/0.033 | 1.43/1.93 | ~0.96 | ✅ |

\* 重噪声下 `min‖A−B‖` 由带噪估计给出，会虚低，非真碰撞。

**M6 SITL 蒙特卡洛**（`report/m6_sitl_mc.md`，每档 N=5，换种子）

| 配置 | 加压项 | 成功率 | 水平误差(成功均值) | 接触速度(成功均值) | min A-B 最低 |
|---|---|---|---|---|---|
| C0_nominal | 标称 | **5/5** | 0.032 m | 2.02 m/s | 0.949 m |
| C1 | 重相对定位噪声 + EMA 滤波 | **4/5** | 0.044 | 1.59 | 0.656* |
| C2 | 同上，不滤波 | **1/5** | 0.049 | 0.06 | 0.427* |
| C3 | 释放误差 σ=0.20 + 载荷闭环 | **5/5** | 0.131 | 1.23 | 0.935 |
| C4 | σ=0.20，不跟踪 | **1/5** | 0.120 | 2.05 | 0.940 |
| C5 | σ=0.30 + 载荷闭环 | **5/5** | 0.131 | 0.93 | 0.955 |

**当前 GUI 实测样例**（`run_m6_gui.sh`，载荷已改为 0.06 m 立方体）
```
双机 READY ~6s
B: CLIMB → WAIT_A (A_alt=4.36, clear=0.87) → TRANSLATE → ALIGN (rel_xy=0.041m) → release
PAYLOAD RELEASED at t=19.340s
B: DIVE plan t_c=0.413s v_rel=2.814 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz=0.025m rel_v=1.932m/s
A/B 各自着陆 @[-3.76,0,-4.49] / [4.76,-0.01,-3.2]
px4_0 / px4_1: Armed + Takeoff detected
```

**M6-moving 编队同速投放（新增）**（`FORMATION_VEL=0.5,0.0,0.0`）
```
B: TRANSLATE done → ALIGN (rel_xy≈0.02m)
B: ALIGNED → /formation/start（编队同速）
A: /formation/start → 编队巡航 vel=[0.5 0.0]
B: FORMATION aligned rel_xy≈0.02m rel_vxy≈0.03 a_spd≈0.55 → release
PAYLOAD RELEASED（A 已巡航 ~2m）
payload: 已与 A 分离（继承 A 速度）
B: DIVE plan t_c≈0.55s v_rel≈3.7 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz≈0.08m rel_v≈2.7m/s
物块停在漏斗上（z≈-3.8）随 B 飞向落点，双机分开落地
```
说明：两机先到同一投影点悬停，再同向同速（0.5 m/s）巡航，运动中释放成功捕获；
物块真实继承 A 的水平速度（Gazebo `DetachableJoint` 分离）。编队模式几何自动设为
`A=5.0m / B=3.3m / RELEASE_Z=0.45m`（偏移要大于 A 起落架、A/B 间距要留下落高度）。

**重复性（无窗口 SITL，`FORMATION_VEL=0.5`）：连续 3 次全部捕获**

| 次数 | 编队对齐 | 捕获 | failsafe | 接住后 |
|---|---|---|---|---|
| 1 | rel_xy=0.067 rel_vxy=0.061 | `horiz=0.114m` | 0/0 | 物块 z=-3.63 随 B 落地 |
| 2 | rel_xy=0.017 rel_vxy=0.187 | `horiz=0.108m` | 0/0 | 物块 z=-3.67 随 B 落地 |
| 3 | rel_xy=0.011 rel_vxy=0.150 | `horiz=0.128m` | 0/0 | 物块 z=-3.71 随 B 落地 |

- 捕获水平偏差 0.108–0.128m vs 有效半径 0.14m，余量偏小（物理盘半径实为 0.20m）。
- GUI 可视化多数成功但偶发 A 端 `Failsafe activated`（渲染负载→仿真时间跳变→飞控 failsafe，
  平台级问题，与算法无关）。Gazebo GUI 默认不跟随相机（固定全场视角）。
> 早期版本“接不住”的四个原因（捕获误触发、DIVE 等待回原点、物块碰 A 起落架被弹飞、横向增益振荡）
> 已全部修复，详见 `SIM_COMMANDS.md` §4.1b。

**M1 GUI 实测样例**（`run_m1_gui.sh`）
```
B: PLAN ok t_r=2.45s p_c=[0,0,-2.78] v_p=[0,0,2.06] h_c=2.78
*** CAPTURED *** d=0.384m rel_v=2.493m/s
```

### 3.4 汇总表：里程碑与状态

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 骨架 + 载荷模型 + 会合规划 + 离线闭环 | ✅ |
| M1 | SITL 端到端（A 悬停释放 / B 会合） | ✅ |
| M2 | A 带速抛投（0.5/1.0/2.0 m/s + 斜向） | ✅ 离线全 PASS |
| M3 | 闭环重规划 | ✅ 释放误差 σ=0.2：7/12→12/12；风+阻力：0/10→10/10 |
| M3+ | acados 终端 MPC + 与 PD 对照 | ✅ 能接，未超过 PD |
| M4 | KF 估计 + 延迟/丢包 + MC | ✅ 估计误差降 2–4×，成功率无提升 |
| M5 | SITL 难度扫描找边界 | ✅ 横向≤0.8 m 可靠；v_p≈4.9 触发 failsafe |
| M6 | 垂直堆叠投放（离线） | ✅ 200/200 |
| M6-SITL | 5 m 起飞→对正→释放→下潜→漏斗捕获→双机分开落地 | ✅ `STACK CAPTURED` |
| M6 sweep/MC | SITL 难度扫描(10 档) + 蒙特卡洛(N=5/档) | ✅ 标称 5/5；滤波/闭环必要性量化 |
| M6-moving | 编队同速投放（同一投影点→同向同速巡航→运动中释放，物块继承 A 速度） | ✅ 无窗口 SITL 3/3（`STACK CAPTURED horiz 0.108–0.128m`）；GUI 偶发平台 failsafe |
| 真机/真漏斗/吸附机构 | — | ❌ 未做 |

---

## 4. 关键发现与负结果

> 负结果与正结果同样重要——它们界定了几何/判据/估计各自的作用边界。

1. **闭环重规划是抗释放误差与模型失配的关键**（M3）：开环 7/12→闭环 12/12；失配下 0/10→10/10。
   释放误差是"已知的系统性偏差"，滚动重规划最划算。

2. **负结果 A：终端 MPC 没有赢过带解析前馈的 PD**（M3+）。在双积分器 + 解析 min-energy 参考下，
   PD 前馈已接近最优；MPC 在短下落场景反而"抄近路"偏离参考，捕获略松。

3. **负结果 B：过冲不是缺陷，而是反馈余量的副产品**（M3+ 参考设计）。
   min-energy 三次在时间充裕时只用 ~74% 的 `a_max`，先爬升再俯冲；强行用
   `w_overshoot`/`staged` 去掉过冲会用满加速度、**丢失反馈余量** → `σ=0.15` 成功率 8/10→6/10。
   **默认保留 `cubic`**；真正的改进方向是**会合几何**（给 B 下滑跑道），而非只改参考。

4. **负结果 C：更好的估计器不提升端到端成功率**（M4）。KF 估计误差降 2–4 倍，但真值基线同样 ~92%
   ⇒ **瓶颈是释放误差 + 动力学限幅，不是估计精度**。估计的价值要等跟踪/判据成为瓶颈时才体现。

5. **M6 的物理下界决定判据**：`v_rel = √(2(g−a_dive)gap)` 说明"B 只能往下压"，
   纯垂直下落的接触速度存在下界；因此**必须用漏斗物理判据**（`v_retain`），固定 `v_c=1.5` 会误杀可接工况。

6. **相对定位在重噪声下必须低通，载荷闭环在释放误差大时是决定性的**（M6 sweep/MC）：
   - 重噪声下 **滤波 4/5 vs 不滤波 1/5**（不滤波对正门限被噪声卡死）；
   - `σ=0.20` 释放误差下 **跟踪载荷 5/5 vs 不跟踪 1/5**（不跟踪会接空）。

7. **SITL 的失败往往在飞控/USV 层而非算法层**（M5）：`v_p≈4.9 m/s` 时 B 姿态/EKF 触发 failsafe，
   而算法本身并未报错。系统边界由**平台鲁棒性**定义。

8. **仿真—现实的差距主要在感知与末端机构**：本项目相对导航仍是"真值 + 噪声"，
   捕获仍是"软件判据 + 实心圆锥盘"，这是最需要推进的两块。

---

## 5. 局限与下一步

### 5.1 当前局限

- **末端机构**：`models/x500_funnel` 是**实心圆锥宽口朝上（≈平顶盘）**，靠低恢复系数"砸住"，
  不是空心导向漏斗；没有夹持/锁定，"接住→带走"靠摩擦与漏斗几何。
- **相对导航**：mesh 替身（A 广播真值 + 噪声/延迟/丢包），非真实视觉/UWB。
- **捕获判定**：M1–M4 用软件判据 + 真值；无真实接触动力学闭环。
- **安全层**：重噪声下 `min A-B` 估计不可靠，缺不确定性裕度；未接住时载荷自由落体砸地，无应急。
- **统计**：SITL 每档 N=5，置信区间宽（±~20%）；边界（σ≥0.4、丢包≥0.5）未扫。
- **平台**：B 有固有健康告警（`Preflight Fail: Attitude failure (roll)`，不影响任务）；
  高动态下 B 的 EKF 鲁棒性不足。

### 5.2 下一步候选（按价值排序）

1. **真正的空心导向漏斗 + 保持机构**（内壁网格 + 低回弹底板，或夹爪/磁吸），
   把"砸住"升级为"导向—夹持—带走"。
2. **相对定位真实化**：接 UWB/视觉/事件相机，替换 `b_node._relnav_a`（接口已隔离）。
3. **推高速度边界**：修 B 的传感器/EKF 鲁棒性（mag 优先级、无 mag 的 yaw 源），
   或把末端俯冲做平滑，把 ~2 m/s 的可靠边界推向 ~4–5 m/s。
4. **不确定性感知安全层**：`min_ab_gap + kσ`，并加丢失目标/接空的应急策略。
5. **更严谨的指标**：记录"捕获前一步真实水平偏差峰值/载荷最终落点"，替代饱和在门限的误差。
6. **真机化**：把已验证的 SITL 配置搬到真机（`report/env_bringup.md` 有 PX4 改动与回滚清单）。
7. **会合几何优化**：给 B 天然下滑跑道（更高待命点），兼顾精度与反馈余量。

---

## 6. 参考文献（代表性）

> 以下为定位方向用的代表工作，便于检索；具体版本/编号建议按题名复核。

**空中操纵 / 抓取 / 停栖**
1. F. Ruggiero, V. Lippiello, A. Ollero, "Aerial Manipulation: A Literature Review," *IEEE RA-L*, 3(3):1957–1964, 2018.
2. A. Ollero, M. Tognon, A. Suarez, D. Lee, A. Franchi, "Past, Present, and Future of Aerial Robotic Manipulators," *IEEE T-RO*, 38(1):626–645, 2022.
3. S. Kim, S. Choi, H. J. Kim, "Aerial manipulation using a quadrotor with a two DOF robotic arm," *IROS*, 2013.
4. J. Thomas, G. Loianno, K. Sreenath, V. Kumar, "Toward image based visual servoing for aerial grasping and perching," *ICRA*, 2014.
5. W. R. T. Roderick, M. R. Cutkosky, D. Lentink, "Bird-inspired dynamic grasping and perching in arboreal environments," *Science Robotics*, 6(61), 2021.
6. J. Moore, R. Cory, R. Tedrake, "Robust post-stall perching with a simple fixed-wing glider using LQR-Trees," *Bioinspiration & Biomimetics*, 9(2), 2014.

**空中抛接 / 动态捕获**
7. R. Ritz, M. W. Müller, M. Hehn, R. D'Andrea, "Cooperative quadrocopter ball throwing and catching," *IROS*, 2012.
8. M. W. Müller, S. Lupashin, R. D'Andrea, "Quadrocopter ball juggling," *IROS*, 2011.

**轨迹生成 / 最优控制 / 工具**
9. D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors," *ICRA*, 2011.
10. C. Richter, A. Bry, N. Roy, "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments," *ISRR*, 2013.
11. R. Verschueren, G. Frison, D. Kouzoupis, et al., "acados—a modular open-source framework for fast embedded optimal control," *Mathematical Programming Computation*, 14:147–183, 2022.
12. W. H. Clohessy, R. S. Wiltshire, "Terminal guidance system for satellite rendezvous," *Journal of the Aerospace Sciences*, 27(9):653–658, 1960.

**制导 / 拦截**
13. P. Zarchan, *Tactical and Strategic Missile Guidance*, AIAA (proportional navigation 及其变体).

**空中回收 / 精确空投（工程案例，供背景）**
14. DARPA Gremlins 项目与 Aurora Flight Sciences "SideArm" 发射回收系统（公开资料）。
15. 样品返回舱空中回收：NASA Genesis（2004）/ Stardust（2006）直升机中空钩取（公开资料）。
16. 精确空投：美国 JPADS（Joint Precision Airdrop System）翼伞制导空投（公开资料）。

**相对导航 / 估计 / 感知**
17. 视觉/事件相机高速目标跟踪与位姿估计综述性文献（按 "event-based object tracking", "relative navigation UWB" 检索）。

---

### 附：项目内相关文档

| 文档 | 内容 |
|---|---|
| `README.md` | 设计、算法、里程碑、结果总览 |
| `MEMORY.md` | 项目记忆：环境事实/坑/决策/下一步 |
| `SIM_COMMANDS.md` | 全部仿真运行命令速查 |
| `report/env_bringup.md` | PX4/ROS/Gazebo 环境打通记录 |
| `report/m5_sitl_results.md` | M5 SITL 难度扫描 |
| `report/m6_sitl_results.md` | M6 SITL 难度扫描（10 档） |
| `report/m6_sitl_mc.md` | M6 SITL 蒙特卡洛（N=5/档） |
| `config/catch_scenarios.yaml` | 单一真值源 |
