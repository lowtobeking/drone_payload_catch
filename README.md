# drone_payload_catch · 空投—空中捕获

无人机 **A** 携载重物，在算法求出的**释放点/释放时刻**抛投；无人机 **B** 实时机动，
在空中的**空间目标点**接住重物（**位置必须到达，相对速度尽量小**）。

复用既有 PX4 v1.14 + ROS 2 + acados 栈的坐标约定、QoS/话题命名、`scenarios.yaml`
单一真值源、安全滤波与诊断工具。核心算法层（载荷抛体 / 会合规划 / 离线仿真）
**刻意不依赖 ROS**，可在任何机器上一条命令验证，再接 SITL。

> 坐标：世界系 **NED**（x=北, y=东, z=下），高度(离地) = `-z`。

## 任务与算法

**问题**：A 悬停（第一版）释放后，载荷做抛体运动；规划器同时决定
`t_r`（释放时刻）与**捕获点**（沿载荷轨迹），使 B 从待命点到会合点且终端速度尽量等于载荷速度。

**难点**：捕获点选得高 → 载荷慢、速度易匹配，但留给 B 的时间短；选得低 → 时间足，
但载荷快（可能超 B 限速）。规划器就在这个权衡上取最优。

**解法（两层）**：

1. **协调（外层）**：网格搜索 `(捕获高度 h_c, 释放时刻 t_r)`。每个候选解析求出 B 的会合轨迹。
2. **会合轨迹（内层）**：双积分器、两端位置/速度、`min ∫|a|²` 的解析解是三次多项式；
   若终端速度精确匹配不可行（如载荷速度 > B 限速），退为**终端速度软代价**的解析解
   （位置仍硬到，速度按权重折中）。
3. **判定**：`|p_B−p_p| < r_c` 且 `|v_B−v_p| < v_c` 即捕获。

## 快速开始（离线，不需要 ROS/SITL）

```bash
cd ~/drone_payload_catch

# 单元自测
python3 -m payload_catch.payload_model
python3 -m payload_catch.rendezvous

# 跑标称工况并出报告
python3 tools/offline_run.py
python3 tools/offline_run.py --all
python3 tools/offline_run.py --plot            # 另存 report/figures/offline_*.png
python3 tools/offline_run.py --sweep-noise     # 释放/B 初值噪声鲁棒性扫描
```

当前标称结果（`M1_basic`，A 悬停 2.5 m，B 待命 (1.2, 0, −3.5)）：

```
规划: t_r=2.34s  t_c=2.66s  h_c=2.00m  v_p=3.13 m/s  速度失配=0
仿真: PASS  最近距离=0.146m  峰值|a|=3.14 m/s²
```

## 配置（单一真值源）

`config/catch_scenarios.yaml` 驱动全部几何 / 参数 / 工况：

- `defaults`：重力、控制频率、载荷质量与（可选）阻力、B 的 `max_speed/max_accel`、
  捕获阈值 `r_c/v_c`、规划器搜索范围与代价权重、风。
- `layouts`：A 悬停点 `a_hover`、B 待命点 `b_standby`（NED）。
- `scenarios`：`M1_basic` / `M1_wind` 等。
- `thresholds`：离线报告判定阈值。

## 目录

| 路径 | 说明 |
|---|---|
| `payload_catch/payload_model.py` | 载荷抛体模型（无阻力解析；可选 linear/quadratic 阻力 + 风） |
| `payload_catch/rendezvous.py` | 协调求解 + 三次多项式会合参考（纯数学） |
| `payload_catch/sim_core.py` | 离线闭环仿真（B 双积分器 + PD 跟踪 + 捕获判定） |
| `tools/offline_run.py` | 离线体检报告 CLI（`--plot` / `--sweep-noise`） |
| `config/catch_scenarios.yaml` | 单一真值源 |
| `launch/`、`payload_catch/*_node.py` | ROS 2 / SITL 接入（M1 完成） |
| `report/` | 报告与出图 |

## 路线图

- [x] **M0** 项目骨架 + 载荷模型 + 会合规划 + 离线闭环（标称必中、噪声扫描）
- [ ] **M1** ROS 2 节点：载荷状态源 / 规划器 / B 控制器 / 捕获监控 / A 悬停释放；
      接 PX4 SITL 双机
- [ ] **M2** 协调联调 + A 带速飞行抛投
- [ ] **M3** B 的终端约束 MPC（acados）+ 载荷状态闭环
- [ ] **M4** 风 / 释放误差 / 状态噪声鲁棒性 + 指标统计
- [ ] **M5** Gazebo 高保真捕获机构 + 安全层 + 真机化

## 开发约定

- 核心算法层（`payload_model` / `rendezvous` / `sim_core`）**不引入 ROS 依赖**，
  保证可离线验证与移植到 Simulink 等。
- 改 `config/catch_scenarios.yaml` 后离线直接生效；ROS launch 读安装副本，需 `colcon build`。
- 所有几何/阈值只写在 yaml，不在代码里硬编码。
