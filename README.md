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

**M3 闭环重规划**（开环 vs 闭环，每档 12 次）：

- **释放误差**（高抛，下落 ~0.8 s）：`σ=0.20 m` 时 开环 **7/12** → 闭环 **12/12**
  （带 0.05 m 载荷测量噪声仍 12/12）。
- **模型失配**（风 [2,1,0] m/s + 线性阻力 k=2，规划器仍按无阻力预测）：
  开环 **0/10** → 闭环 **10/10**（`replan_dt=0.3s` 即可）。

结论：闭环重规划能显著提升释放误差与模型失配下的捕获成功率。

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
| `payload_catch/sim_core.py` | 离线闭环仿真（A 恒速飞行 + B 双积分器 PD + 捕获判定 + 闭环重规划） |
| `tools/offline_run.py` | 体检报告 CLI（`--plot` / `--sweep-noise` / `--compare`） |
| `config/catch_scenarios.yaml` | 单一真值源 |
| `env.sh` | 环境变量（acados/ROS/RMW/PX4 SITL） |
| `report/env_bringup.md` | B 阶段环境打通记录（含 PX4 检出问题与回滚清单） |
| `launch/`、`payload_catch/*_node.py` | ROS 2 / SITL 接入（M1 待环境修复） |

## 路线图

- [x] **M0** 项目骨架 + 载荷模型 + 会合规划 + 离线闭环
- [x] **M2** A 带速飞行抛投（恒速直线，含斜向）
- [x] **M3** 闭环重规划（`solve_inflight` + 载荷状态噪声），开环 vs 闭环对比
- [~] **B** 环境打通：`px4_msgs` 已修正、DDS 通；传感器桥 `Gyro STALE` 待解
- [ ] **M3+** B 的终端约束 MPC（acados，可选）
- [ ] **M1** ROS 2 节点：载荷状态源 / 规划器 / B 控制器 / 捕获监控 / A 悬停释放
- [ ] **M4** 更完整鲁棒性（延迟、丢包、估计滤波）+ 指标统计
- [ ] **M5** Gazebo 高保真捕获机构 + 安全层 + 真机化

## 开发约定

- 核心算法层（`payload_model` / `rendezvous` / `sim_core`）**不引入 ROS 依赖**，
  保证可离线验证与移植。
- 改 `config/catch_scenarios.yaml` 后离线直接生效；ROS launch 读安装副本，需 `colcon build`。
- 所有几何/阈值只写在 yaml，不在代码里硬编码。
