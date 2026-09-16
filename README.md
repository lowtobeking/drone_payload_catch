# drone_payload_catch · 空投—空中捕获

无人机 **A** 携载重物飞行，在算法求出的**释放时刻/会合点**抛投；无人机 **B** 实时机动，
在空中的**空间会合点**接住重物（**位置必须到达，相对速度尽量小**）。

复用既有 PX4 + ROS 2 + acados 栈的坐标约定、QoS/话题命名、`yaml` 单一真值源、安全滤波
与诊断工具。核心算法层（载荷抛体 / 会合规划 / 离线闭环仿真）**刻意不依赖 ROS**，
可在任何机器上一条命令验证。

> 坐标：世界系 **NED**（x=北, y=东, z=下），高度(离地) = `-z`。

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

## 目录

| 路径 | 说明 |
|---|---|
| `payload_catch/payload_model.py` | 载荷抛体模型（无阻力解析；可选 linear/quadratic 阻力 + 风） |
| `payload_catch/rendezvous.py` | 协调求解 `(t_r,τ_c)` + 三次多项式会合参考 + 软终端速度 + 闭环 `solve_inflight` |
| `payload_catch/sim_core.py` | 离线闭环仿真（A 恒速飞行 + B 控制 + 捕获判定 + 闭环重规划） |
| `payload_catch/mpc_terminal.py` | B 的 acados 终端（会合）MPC |
| `payload_catch/payload_filter.py` | 载荷状态估计（卡尔曼滤波 / 朴素对照） |
| `tools/offline_run.py` | 体检报告 CLI（`--plot` / `--sweep-noise` / `--compare`） |
| `config/catch_scenarios.yaml` | 单一真值源 |
| `env.sh` | 环境变量（acados/ROS/RMW/PX4 SITL） |
| `report/env_bringup.md` | B 阶段环境打通记录（含 PX4 检出问题与回滚清单） |
| `launch/`、`payload_catch/*_node.py` | ROS 2 / SITL 接入（M1 待环境修复） |

## 路线图

- [x] **M0** 项目骨架 + 载荷模型 + 会合规划 + 离线闭环
- [x] **M2** A 带速飞行抛投（恒速直线，含斜向）
- [x] **M3** 闭环重规划（`solve_inflight` + 载荷状态噪声），开环 vs 闭环对比
- [x] **M3+** B 的终端 MPC（acados），与 PD 对照
- [~] **B** 环境打通：`px4_msgs` 已修正、DDS 通；传感器桥 `Gyro STALE` 待解
- [ ] **M1** ROS 2 节点：载荷状态源 / 规划器 / B 控制器 / 捕获监控 / A 悬停释放
- [ ] **M4** 更完整鲁棒性（延迟、丢包、估计滤波）+ 指标统计
- [ ] **M5** Gazebo 高保真捕获机构 + 安全层 + 真机化

## 开发约定

- 核心算法层（`payload_model` / `rendezvous` / `sim_core`）**不引入 ROS 依赖**，
  保证可离线验证与移植。
- 改 `config/catch_scenarios.yaml` 后离线直接生效；ROS launch 读安装副本，需 `colcon build`。
- 所有几何/阈值只写在 yaml，不在代码里硬编码。
