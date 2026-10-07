# 仿真运行命令速查

> 本项目所有仿真的运行命令汇总。所有命令都在 **WSL Ubuntu-24.04** 内执行。
> 项目路径：`~/drone_payload_catch`
> 从 Windows 侧访问：`\\wsl.localhost\Ubuntu-24.04\home\caolihao\drone_payload_catch`

---

## 0. 环境准备

```bash
cd ~/drone_payload_catch

# 一键环境（PX4-1.16 + px4_msgs v1.16.2 + acados + rmw_fastrtps + Gazebo 资源路径）
source ~/drone_payload_catch/env.sh
```

> **注意**：离线仿真（纯 Python）不需要 `source env.sh`；
> 但**涉及 acados MPC 的工况**（`M3p_*`）需要手动补 acados 路径，否则报
> `OSError: libqpOASES_e.so: cannot open shared object file`：

```bash
export ACADOS_SOURCE_DIR=/home/caolihao/drone_package_20260908/acados
export LD_LIBRARY_PATH=/home/caolihao/drone_package_20260908/acados/lib:$LD_LIBRARY_PATH
```

---

## 1. 离线单元自测（不需要 ROS / SITL）

```bash
python3 -m payload_catch.payload_model   # 载荷抛体模型
python3 -m payload_catch.rendezvous      # 会合规划
python3 -m payload_catch.sim_core        # 离线闭环仿真
```

---

## 2. 离线体检报告（`tools/offline_run.py`）

```bash
# 跑单个工况（默认 M1_basic）
python3 tools/offline_run.py

# 指定工况
python3 tools/offline_run.py --scenario M1_basic
python3 tools/offline_run.py --scenario M2_line_v10
python3 tools/offline_run.py --scenario M3_high

# 跑全部工况（含 MPC，需先设 acados 环境变量，见 §0）
python3 tools/offline_run.py --all

# 生成轨迹图（PNG → report/figures/offline_*.png）
python3 tools/offline_run.py --plot
python3 tools/offline_run.py --scenario M2_crosswind --plot

# 开环 vs 闭环（释放误差 / 测量噪声）
python3 tools/offline_run.py --scenario M2_line_v10 --compare

# 释放/B 初值噪声鲁棒性扫描
python3 tools/offline_run.py --sweep-noise

# PD vs 终端 MPC 控制器对比
python3 tools/offline_run.py --controller-compare

# 蒙特卡洛：测量严重度 × 估计器(none/naive/kf)
python3 tools/offline_run.py --scenario M4_high_kf --mc
```

---

## 3. M6 垂直堆叠投放离线（`tools/stack_run.py`）

> ⚠️ `M6_stack_drop` 在 §2 的 3D 通用规划器里会 FAIL，这是**预期行为**；
> 垂直堆叠场景必须用独立的解析规划器，即本脚本。

```bash
python3 tools/stack_run.py            # 单次
python3 tools/stack_run.py --sweep-dive   # 扫 a_dive：冲击 vs 刹车余量
python3 tools/stack_run.py --sweep-gap    # 扫 gap：接触速度 vs v_retain
python3 tools/stack_run.py --mc 200       # 蒙特卡洛（相对定位噪声/延迟）
python3 tools/stack_run.py --scenario M6_stack_bigfunnel --mc 100  # 大漏斗(0.30m)：捕获余量↑14×

# 鲁棒几何优化 / 末端机构建模（见 report/robust_geometry_and_retention.md）
python3 tools/geom_opt.py --wind 3.0        # 干扰下最大化最坏情况余量的几何搜索
python3 tools/funnel_model.py               # 平顶盘 vs 空心锥 vs 主动保持
python3 tools/stack_run.py --scenario M6_stack_nodive    # 小 gap + 免下潜（横风鲁棒）
python3 tools/drag_reject.py                # 抗阻力/风扰（二次阻力、阵风）× 估计器
python3 tools/stack_run.py --scenario M6_stack_windcomp --mc 100   # A 端迎风预补偿（强侧风 100/100）
python3 tools/stack_run.py --scenario M6_stack_lock --mc 100       # 主动保持锁扣（下击暴流 100/100）

# 干扰鲁棒性（侧风）：原始追尾 vs 速度前馈 vs 预测式对正（见 report/robustness_wind.md）
python3 tools/stack_run.py --scenario M6_stack_wind            # 侧风单测（默认含速度前馈）
python3 tools/stack_run.py --scenario M6_stack_wind --lead 1.0 # 加预测式对正
python3 tools/stack_run.py --sweep-wind                        # 侧风扫掠（两种干扰制度）
python3 tools/stack_run.py --sweep-wind --wind-k 2.0           # 更强阻力下的扫掠

# 估计器 / 垂直闭环 / 漏斗几何（见 report/m6_robustness_opt.md）
python3 tools/stack_run.py --sweep-funnel                      # 扫口半径(eff_r)/深度/e(v_retain)/机械臂
python3 tools/stack_run.py --scenario M6_stack_wind --est kf --lead 1.0
python3 tools/stack_run.py --scenario M6_stack_wind --est windkf --lead 1.0   # 增广风估计KF(推荐)
python3 tools/stack_run.py --scenario M6_stack_wind --est windkf --vert adaptive
```

---

## 4. 可视化仿真（Gazebo）

> 本项目「可视化仿真」= **Gazebo 3D 仿真（SITL）**，Gazebo 窗口经 WSLg 显示到 Windows 桌面。
> 前置：`source ~/drone_payload_catch/env.sh` 已执行、PX4-1.16 已编译、本项目 ROS 工作区已 `colcon build`。

### 4.1 M6 垂直堆叠 + 漏斗捕获（推荐先看这个）

流程：A、B 相距 5m 起飞 → B 爬到 A 严格正下方对正 → A 释放载荷（橙色方块）→
B 温和下潜用刚性漏斗接住 → 保持 6s 后双机分开落地。

```bash
cd ~/drone_payload_catch
bash run_m6_gui.sh [观察秒数，默认 110]
```

可调参数（环境变量）：

| 变量 | 默认 | 含义 |
|---|---|---|
| `A_HOVER` | `0.0,0.0,-4.5` | A 悬停点（NED） |
| `B_STANDBY` | `0.0,0.0,-3.5` | B 待命点（NED） |
| `B_OFFSET` | `5.0,0.0,0.0` | 双机水平间距（NED） |
| `B_POSE_ENU` | `0,5.0,0,0,0,0` | B 在 Gazebo 的初始位姿（ENU） |
| `RELEASE_OFFSET` | `0.0,0.0,0.15` | 载荷生成在 A 下方偏移 |
| `RELEASE_Z` | `0.15` | 释放点竖直偏移标量（与 `payload_release_offset` 对应） |
| `FORMATION_VEL` | `0.0,0.0,0.0` | **非零即启用「编队同速投放」**（见 §4.1b） |
| `FUNNEL_MOUTH` | `0.20` | 漏斗口半径；≠0.20 自动选用大漏斗模型 `x500_funnel_big`（见 §4.1c） |
| `FUNNEL_TYPE` | `flat` | `flat`=实心盘；`cup`=空心导向锥杯 `x500_funnel_cup`（见 §4.1d） |
| `PAYLOAD_LOCK` | `0` | `1`=主动保持：捕获时在 B 漏斗处重生成带 `DetachableJoint` 的载荷并锁定（见 `report/active_retention.md`） |

```bash
# 示例：加大落差重跑
A_HOVER="0.0,0.0,-5.0" B_STANDBY="0.0,0.0,-4.0" bash run_m6_gui.sh 110
```

### 4.1b M6-moving 编队同速投放（新增·加难度）

流程：A、B 先到**同一投影点**悬停 → B 发 `/formation/start` → 两机以 `FORMATION_VEL`
**同向同速小速度巡航**（载荷经 `DetachableJoint` 挂在 A 下方随飞）→ B 位置+速度都对正后 →
A 释放（物块**继承 A 速度**）→ B 下潜用漏斗接住 → 双机分开落地。

```bash
cd ~/drone_payload_catch
# 0.5 m/s 向北编队同速投放
FORMATION_VEL="0.5,0.0,0.0" bash run_m6_gui.sh 70
# 1.0 m/s（更难）
FORMATION_VEL="1.0,0.0,0.0" bash run_m6_gui.sh 70
```

要点与坑：
- 非零 `FORMATION_VEL` 会自动改用挂载型载荷 `models/payload_attached`（带 DetachableJoint），
  并自动调整几何：`RELEASE_Z=0.45`、`A_HOVER=0.0,0.0,-5.0`、`B_STANDBY=0.0,0.0,-3.3`（可在外部覆盖）。
- **DetachableJoint 在模型 configure 时即已挂载**：载荷在 A 正下方生成后**不可再发 attach**，
  否则叠加第二个固定关节 → 过约束 → 分离瞬间被甩飞（实测 ~6.8 m/s）。
- 分离时物块继承 A 的速度（物理正确）；B 在 DIVE 阶段跟踪 A（编队模式，无释放误差，更稳）。
- 编队期间 B 高度必须严格 ≤ A−`min_ab_gap`（否则 B 会撞到 A 下方挂的载荷）。

**物块“接不住”排查（已修）**：
1. 捕获判据太宽/误触发（物块还挂在 A 上就判捕获）→ DIVE “重锚”：等物块真正下落（`vz>` `dive_anchor_vz`）
   才下潜，并把捕获窗口收紧到漏斗口上下 `±catch_z_tol`。
2. DIVE 等待期拿 `standby`（原点）当悬停目标，把已在 x≈几米处的 B 往回拉 → 改为**原地悬停**。
3. 物块与 A 起落架净空不足被弹飞 → 释放偏移 0.30→**0.45**，并拉开 A/B 间距。
4. 横向 P 增益 2.5 在相对定位延迟下振荡（速度 0.02↔0.83）→ 降到 **1.2**，
   编队/DIVE 跟踪 A 而非带噪的物块估计。

**当前实测状态（无窗口 SITL `run_m6_sitl.sh`，FORMATION_VEL=0.5）**：

| 次数 | 编队对齐 | 捕获 | A/B failsafe | 接住后 |
|---|---|---|---|---|
| 1 | rel_xy=0.067 rel_vxy=0.061 | `STACK CAPTURED horiz=0.114m` | 0/0 | 物块 z=-3.63 随 B 落地 |
| 2 | rel_xy=0.017 rel_vxy=0.187 | `STACK CAPTURED horiz=0.108m` | 0/0 | 物块 z=-3.67 随 B 落地 |
| 3 | rel_xy=0.011 rel_vxy=0.150 | `STACK CAPTURED horiz=0.128m` | 0/0 | 物块 z=-3.71 随 B 落地 |

- **无窗口 3/3 稳定捕获**；GUI 可视化多数成功但**偶发 A 端 `Failsafe activated`**
  （渲染负载→仿真时间跳变→飞控 failsafe，平台级问题，与算法无关）。
- 捕获水平偏差 0.108–0.128m vs 有效半径 0.14m，**余量偏小**（物理盘半径实为 0.20m）。

**运行日志目录**：无窗口 `~/payload_catch_m6/`；GUI `~/payload_catch_m6_gui/`。

### 4.1d 空心导向锥杯（末端能力实验，含负结果）

```bash
FUNNEL_TYPE=cup bash run_m6_sitl.sh 70       # x500_funnel_cup
bash tools/funnel_drop_test.sh               # 倾斜平台投放：杯 vs 平盘（可 TILT=0.436 OFFX=0.15）
```

结果与结论见 `report/hollow_funnel_cup.md`：SITL 可捕获，但**敞口杯在倾斜 >25° 时不如高摩擦平盘（45°）**——
杯的价值在“导向/侧向兜接”，保持/倾角需主动锁扣。

### 4.1c 大漏斗（末端能力，新增）

用 `FUNNEL_MOUTH` 切换漏斗口半径；≠0.20 时自动用 `models/x500_funnel_big`（口半径 0.30m，
`eff_r 0.15→0.25m`）。动机与结果见 `report/end_effector_bigfunnel.md`。

```bash
cd ~/drone_payload_catch
FUNNEL_MOUTH=0.30 bash run_m6_sitl.sh 70    # 无窗口
FUNNEL_MOUTH=0.30 bash run_m6_gui.sh 110    # GUI
# 加相对定位噪声：
FUNNEL_MOUTH=0.30 LAUNCH_EXTRA="rel_pos_sigma:=0.10 rel_latency:=0.10 rel_bias:=0.05" \
  bash run_m6_sitl.sh 70
```

**实测**（`~/payload_catch_m6/launch.log`）：
- 标称：`STACK CAPTURED horiz=0.028m rel_v=2.193m/s`，两机落地；
- 加噪（σ0.10/延迟0.10/偏置0.05）：`STACK CAPTURED horiz=0.030m rel_v=2.174m/s`，无 failsafe。

离线：`python3 tools/stack_run.py --scenario M6_stack_bigfunnel --mc 100`
→ 捕获余量 min **0.0076→0.1076m（14×）**。

### 4.2 M1 水平会合捕获（acados MPC）

流程：A 悬停释放 → B 用 acados 终端 MPC 跟踪会合参考机动 → 空中接住载荷。

```bash
cd ~/drone_payload_catch
bash run_m1_gui.sh [观察秒数，默认 110]
```

可调参数（环境变量）：

| 变量 | 默认 | 含义 |
|---|---|---|
| `A_HOVER` | `0.0,0.0,-3.0` | A 悬停点（NED） |
| `B_STANDBY` | `3.0,0.0,-3.0` | B 待命点（NED） |
| `B_OFFSET` | `3.0,0.0,0.0` | 双机水平间距 |
| `B_POSE_ENU` | `0,3.0,0,0,0,0` | B 初始位姿（ENU） |
| `CTRL` | `mpc` | B 控制器（`mpc` / `pd`） |

```bash
# 示例：换 PD 控制器重跑
CTRL=pd bash run_m1_gui.sh 110
```

### 4.3 脚本内部流程（两个 GUI 脚本一致）

1. 清理旧进程（`pkill px4 / gz sim / MicroXRCEAgent / 节点`）→ 清 `/dev/shm/fastrtps_*`
2. 启动 Gazebo GUI（`gz sim -r $SITL_WORLD`）→ 等 20s
3. 启动 MicroXRCEAgent + 2×PX4（A / B）
4. 等双机日志出现 `Ready for takeoff`
5. `ros2 launch` 启动任务节点，循环打印阶段进展
6. 打印捕获结果 → cleanup 杀掉所有进程

### 4.4 关键日志位置

| 场景 | 日志目录 | 关键文件 |
|---|---|---|
| M6 GUI | `~/payload_catch_m6_gui/` | `launch.log` / `gz.log` / `agent.log` |
| M1 GUI | `~/payload_catch_gui/` | `launch.log` / `gz.log` / `agent.log` |
| PX4 双机 | `~/px4_logs/` | `px4_0.log`（A）/ `px4_1.log`（B） |

### 4.5 无窗口版（同任务，不弹 Gazebo 窗口）

```bash
source ~/drone_payload_catch/env.sh
bash ~/drone_payload_catch/run_m6_sitl.sh 70               # M6 垂直堆叠
COORD=handshake bash ~/drone_payload_catch/run_m6_sitl.sh 70   # 协同释放握手
# 编队握手 + 意图升级（预测落点）：
COORD=handshake FORMATION_VEL="0.5,0.0,0.0" bash ~/drone_payload_catch/run_m6_sitl.sh 75
COORD=handshake WIND_EST="0.0,0.0,0.0" bash ~/drone_payload_catch/run_m6_sitl.sh 70
# 安全层（keep-out=min_ab_gap+2σ）/ PX4 EKF 风估计：
COORD=handshake SAFETY_FLOOR=0.20 bash ~/drone_payload_catch/run_m6_sitl.sh 70
COORD=handshake PX4_WIND=1 bash ~/drone_payload_catch/run_m6_sitl.sh 70

# 协同协议 SITL 验证（多组配置 + 不变量检查）→ report/coordination_validation.md
python3 tools/validate_coord.py

# M6-moving（边飞边接）验证：默认测支持速度 0.5/1.0 m/s，检查捕获/保持/双机落地/无 failsafe
python3 tools/validate_m6_moving.py                 # 默认 0.5,1.0
python3 tools/validate_m6_moving.py --speeds 0.5,1.0,2.0 --reps 3

# 控制/规划旋钮：ZEM 终端导引增益；及时释放；超时中止
COORD=handshake ZEM=1.0 bash ~/drone_payload_catch/run_m6_sitl.sh 70
FORMATION_VEL="1.0,0.0,0.0" LAUNCH_EXTRA="formation_timeout_s:=12.0 align_reset_tol:=0.25" \
  bash ~/drone_payload_catch/run_m6_sitl.sh 100

# 安全监督：飞行异常 kill（单机飞行终止）
#   外部 kill：
ros2 topic pub --once /safety/kill_b std_msgs/msg/Bool "{data: true}"     # 只杀 B
#   自动 kill（异常持续）：launch 参数 safety_auto_kill:=true safety_tilt_max_deg:=45
FUNNEL_MOUTH=0.30 bash ~/drone_payload_catch/run_m6_sitl.sh 70   # 大漏斗(0.30m)
bash ~/drone_payload_catch/run_m1_sitl.sh      # M1 水平会合
```

### 4.6 优化第二轮新增旋钮（安全/控制/协同，见 `report/opt_round2.md`）

均通过 `LAUNCH_EXTRA` 传入（A/B 共用；编队可叠 FORMATION_VEL）：

| launch 参数 | 默认 | 作用 |
|---|---|---|
| `safety_pullback_enable` / `safety_geofence_xy` / `safety_geofence_alt` | true / 50 / 30 | 越界**回拉**（而非只 kill） |
| `safety_hold_escalate` / `safety_hold_timeout` | none / 8.0 | 持续 HOLD 后升级 `land` |
| `safety_auto_kill` / `safety_tilt_max_deg` | false / 60 | 临界异常自动飞行终止 |
| `a_ff_gain` | 1.0 | DIVE 加速度前馈（0=关） |
| `sp_rate_limit` | 0.0 | 速度指令变化率限幅 (m/s²) |
| `keepout_dist` / `keepout_gain` | 0.60 / 1.0 | 3D 反应式 keep-out |
| `adaptive_dive` / `adaptive_alt_floor` | false / 0.35 | 在线自适应下潜 |
| `commit_hold_s` | 0.20 | A 释放提交窗口 |
| `use_b_sigma` / `release_sigma_max` | true / 0.15 | A 余量闸用 B 在线 σ |
| `sensor_constraints_enable` | true | 传感器/估计器约束总开关 |
| `sensor_use_ekf_sigma` | true | 用 PX4 `eph/epv` 作位置 σ |
| `sensor_watchdog_enable` | true | 健康/一致性看门狗（valid/dead_reckoning/reset） |
| `sensor_use_est_limits` | true | 用估计器限值 `vxy_max/vz_max/hagl_min` |
| `sensor_eph_max` / `sensor_epv_max` / `sensor_reset_hold_s` | 0.50 / 0.50 / 1.0 | σ 上限 / 跳变保持 |
| `sensor_watchdog_heading` | false | 航向可用性也当硬约束（本仿真常 false） |
| `attitude_constraint_enable` | true | 姿态/角速率安全滤波 |
| `tilt_soft_deg` / `tilt_hard_deg` | 25 / 40 | 倾角软/硬限（度） |
| `rate_soft_dps` / `rate_hard_dps` | 150 / 300 | 角速率软/硬限（deg/s） |
| `accel_h_max` | 5.0 | 水平指令加速度上限 (m/s²) |

```bash
# 示例：自适应下潜 + 释放提交窗口（handshake）
COORD=handshake LAUNCH_EXTRA="adaptive_dive:=true" bash run_m6_sitl.sh 75
# 示例：越界回拉（把高度围栏收到 3m，A/B 会被压回 3m）
LAUNCH_EXTRA="safety_geofence_alt:=3.0" bash run_m6_sitl.sh 40
```

---

## 5. SITL 难度扫描 / 蒙特卡洛（M6）

### 5.1 协同交接基准（研究用，阶段 0）

系统扫描 `coord_mode × intent × σ × delay`，带 Wilson 置信区间，输出 `report/coordination_benchmark.md`：

```bash
python3 tools/bench_coord.py --quick            # 5 配置×1（≈10 min，先验证）
python3 tools/bench_coord.py --grid gate --reps 2   # 证书闸 vs 启发式闸（论文核心消融）
python3 tools/bench_coord.py --grid ablation --reps 3          # 逐项消融（标称）
python3 tools/bench_coord.py --grid ablation --stress --reps 3 # 逐项消融（stress，实验主表）
python3 tools/bench_coord.py --reps 3           # 小网格×3 次
python3 tools/bench_coord.py --full --reps 3     # 全网格 36 配置×3（≈多小时）
python3 tools/bench_coord.py --dry-run          # 只打印配置
python3 tools/bench_coord.py --only auth       # 只跑名字匹配的配置
```

指标：捕获率(95% CI)、horiz、rel_v、min\|A-B\|、就绪→释放时延、协调异常、failsafe。

### 5.2 C1 概率证书（研究 W2）

```bash
python3 tools/coord_prob.py --n 300000      # 证书 k(ε) + 策略/延迟扫描（离线、秒级）
```
输出 → `report/coordination_probability.md`：精确 Rice/非中心卡方证书 `T(ε)`、
`P(capture)≥1−ε`、exact/Chernoff/heuristic 对比、direct vs authority、延迟偏置 `v_A·d`、
EMA 标定（`rel_pos_sigma→σ_e`）、覆盖性检查。

**证书闸接入 SITL**（`release_gate_mode=certificate`）：
> B 的 `/drone_b/ready` 已扩为 `[.., sigma_abs, sigma_rel]`；证书闸用**相对 σ_rel**（启发式用 σ_abs）。

**证书闸接入 SITL 示例**：
```bash
COORD=handshake FUNNEL_MOUTH=0.30 \
  LAUNCH_EXTRA="release_gate_mode:=certificate cert_eps:=0.05 cert_sigma_track:=0.02" \
  bash run_m6_sitl.sh 70
```

### 5.3 C2 协议时序界（研究 W3）

```bash
python3 tools/coord_proto.py --n 200000   # 取消窗口/取消捕获率/设计表/丢包结局
```
输出见 `report/coordination_protocol.md`：`release_lead=d_max/(2(1−catch))`、丢 cmd→未下潜失效。

### 5.4 C4 联合机动（研究 W3）

```bash
python3 tools/coord_maneuver.py --wind 0 --n 12   # A 速度/高度 × B 机动联合优化
```
输出 → `report/coordination_maneuver.md`：速度—余量权衡、联合几何恢复余量、顺风耦合。

### 5.5 C5 handover-CBF（研究 W4）

```bash
python3 tools/coord_cbf.py --n 50000   # 速度级防碰不变集 + 交接门
```
输出 → `report/coordination_cbf.md`：可行域内 CBF 100% 满足（未滤波 56.8%）；可行性 `v_max ≥ ‖v_A‖−(α/2)h/‖r‖`。

### 5.7 理论三条（T1 最优性 / T3 延迟 CBF）

```bash
python3 tools/coord_optimal.py --n 800000 --sigma0 0.25 --eps 0.05   # T1：中心球最优
python3 tools/coord_cbf.py --n 5000                                  # T3：延迟 CBF（naive 违反 vs robust 安全）
```
输出 → `report/coordination_theory.md`：T1 释放域最优性；T2 联合 handover 证书；T3 延迟鲁棒 `ρ=(v_A+v_B)d`。

### 5.6 全套整合验证（C1 证书闸 + C5 CBF + C2 冗余）

**一键预设（推荐）**：`MODE=full` 开启 握手+证书闸+CBF+intent+延迟鲁棒 CBF（默认仍是 baseline）：
```bash
MODE=full bash run_m6_sitl.sh 70          # 研究特性全开（论文主线/演示）
```
等价于：
```bash
COORD=handshake FUNNEL_MOUTH=0.30 \
  LAUNCH_EXTRA="release_gate_mode:=certificate keepout_mode:=cbf use_intent:=true keepout_delay_s:=0.05" \
  bash run_m6_sitl.sh 70
```
- `keepout_mode=cbf`：C5 速度级 CBF；`keepout_delay_s`：**T3 延迟鲁棒收紧**（`d_eff=d_safe+(‖v_A‖+‖v_B‖)d`）。
- C2 冗余：若 `release_cmd` 丢失，B 收 `/payload/released` 也兜底触发 DIVE。
- 实测（`MODE=full`）：`STACK CAPTURED horiz=0.043m`、双机落地、无 failsafe。

```bash
source ~/drone_payload_catch/env.sh
bash tools/sweep_m6_sitl.sh      # 10 档难度扫描 → report/m6_sitl_results.md
NREP=5 bash tools/mc_m6_sitl.sh  # 蒙特卡洛 → report/m6_sitl_mc.md
```

---

## 6. 本次实测记录（2026-09-22，本机跑通）

### 6.1 离线单元自测

```
payload_model 自测通过 (解析/积分 err=2.44e-13, fall_time(1.5m)=0.5530s)
rendezvous: hover feasible t_r=2.35 t_c=2.670 h_c=2.00 v_p=[0 0 3.14] dv=0.000
sim_core:   M1_basic success miss=0.1182m / M2_line_v10 success miss=0.1562m
```

### 6.2 `offline_run.py --all` 全部 PASS

| 工况 | 最近距离 | 备注 |
|---|---|---|
| M1_basic / M1_wind | 0.118 m | 悬停释放 |
| M2_line_v05 / v10 / v20 | 0.160 / 0.156 / 0.150 m | 带速抛投 |
| M2_crosswind | 0.151 m | 斜向 |
| M3_high | 0.283 m | 高抛 |
| M3_wind_drag | 0.117 m | 闭环重规划 |
| M3p_line_mpc / M3p_high_mpc | 0.272 / 0.283 m | acados MPC |
| M4_line_naive / kf / high_kf | 0.149 / 0.149 / 0.274 m | 估计滤波 |
| M6_stack_drop | FAIL（预期） | 用 stack_run.py 跑 |

### 6.3 `stack_run.py --mc 200`

```
成功率 = 200/200 (100%)
最近距离: mean=0.0987m  max=0.1328m
捕获时水平偏差: max=0.0960m (口内有效半径=0.15m)
```

### 6.4 M6 GUI 3D 仿真（`run_m6_gui.sh 110`）

```
双机 READY ~6s
B: CLIMB done → WAIT_A (A_alt=4.36, clear=0.88) → TRANSLATE → ALIGN (rel_xy=0.027m)
B: DIVE plan t_c=0.419s v_rel=2.856 v_retain=4.044 feasible=True
PAYLOAD RELEASED at t=18.830s
*** STACK CAPTURED *** horiz=0.025m rel_v=0.489m/s
两机均 Armed + Takeoff detected
```

### 6.5 M1 GUI 3D 仿真（`run_m1_gui.sh 110`）

```
[prebuild] MPC OK
双机 READY ~12s
B: PLAN ok t_r=2.45s p_c=[0,0,-2.78] v_p=[0,0,2.06] h_c=2.78
B: payload released → rendezvous
*** CAPTURED *** d=0.384m rel_v=2.493m/s
两机均 Takeoff detected
```

---

## 7. 常见坑

1. **acados MPC 报 `libqpOASES_e.so` 找不到** → 设 `ACADOS_SOURCE_DIR` + `LD_LIBRARY_PATH`（§0）。
2. **`offline_run.py --all` 里 M6_stack_drop FAIL** → 不是 bug，改用 `stack_run.py`。
3. **PX4 `Gyro #0 fail: STALE` 永不许解锁** → 必须用 `~/drone_package_20260908/PX4-Autopilot-1.16`
   （main 分支的 x500 模型 IMU 无噪声，已弃用），`env.sh` 已指向正确路径。
4. **GUI 仿真结束时的 `Failsafe activated`** → 是脚本 cleanup 阶段强杀 PX4 触发的，属正常收尾。
5. **话题静默 / 看不到数据** → 检查 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 且 px4_msgs @ v1.16.2。
6. **M6-moving：物块被甩飞 / 接不住** → 见 §4.1b“物块接不住排查”；核心：`DetachableJoint` 不可重复发 attach、
   `RELEASE_Z` 要大于 A 起落架、DIVE 等待要原地悬停、横向增益不能太大。
7. **GUI 偶发 `Failsafe activated`（非收尾）** → Gazebo 渲染负载拖慢实时性→PX4 仿真时间跳变→飞控 failsafe；
   属平台问题，改用无窗口 `run_m6_sitl.sh` 可稳定复现（3/3）。
