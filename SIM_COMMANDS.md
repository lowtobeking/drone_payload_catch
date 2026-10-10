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
> 但**涉及 acados MPC 的工况**（`M3p_*` / `mpc_terminal` / `smoke_acados`）需要 acados 路径，否则报
> `OSError: libqpOASES_e.so: cannot open shared object file`。
> **推荐直接用 `bash tools/run_checks.sh`（会自动 source env.sh），或手动补：**

```bash
export ACADOS_SOURCE_DIR=/home/caolihao/drone_package_20260908/acados
export LD_LIBRARY_PATH=/home/caolihao/drone_package_20260908/acados/lib:$LD_LIBRARY_PATH
```

---

## 0.5 起飞前自检（preflight）

跑 SITL / 真机之前先确认"能不能飞"（构建/依赖/配置/PX4/RMW/ROS），失败非 0。

```bash
python3 tools/preflight_check.py           # 离线静态：配置/依赖/二进制（缺 ROS 只 warn）
python3 tools/preflight_check.py --sitl    # SITL 严格：PX4/world/gz/agent/ROS 缺失即 fail
python3 tools/preflight_check.py --live    # 持续探测运行中的 PX4（见下 live_probe）
python3 tools/preflight_check.py --live --gps   # 室外：加 GPS 严格门（eph≤1.5/epv≤2.5/sats≥20）
python3 tools/preflight_check.py --logs    # 日志门：Ready 必须有；Gyro STALE/Arming denied 致命
```

**飞控参数体检**（对标参考工程 `fc_configure.py`；判据见 `report/fcu_safety.md`）：
```bash
python3 tools/preflight_params.py --file dump.txt --profile indoor     # 从 dump 文件
python3 tools/preflight_params.py --mavlink udpout:127.0.0.1:18570 --profile sitl  # 直连飞控
python3 tools/preflight_params.py --list                              # 列出分组预置
python3 tools/fcu_configure.py -g failsafe,limits_indoor              # 配置器（dry-run，--apply 才写）
```

`--live` 优先调 **`tools/live_probe.py`**（`--seconds` 秒**持续订阅**取聚合，避免 `ros2 topic echo --once` 快照骗人）：
EKF（xy/z/v_valid 比例、非 dead_reckoning、eph/epv）+ `failsafe_flags`（local_position/velocity/attitude/offboard/geofence/critical）
+ `estimator_status_flags`（tilt/yaw 对齐、**磁罗盘在线/故障/受扰**、加计故障 `fs_bad_acc_*`）+ GPS（`vehicle_gps_position`）。
> 注：PX4-1.16 的 uXRCE-DDS **不桥接** `vehicle_imu_status`/`vehicle_magnetometer`，故 IMU/磁健康走 `estimator_status_flags`。无 rclpy 时回退 ros2 CLI 快照。

**上行链路验证**（发 offboard 心跳看 `offboard_control_signal_lost` 翻转；不 ARM/不起飞）：
```bash
python3 tools/uplink_test.py --drone 0 --seconds 5
```

SITL / 真机启动脚本内置可选钩子（**默认不开**，不影响现有流程）：

```bash
PREFLIGHT=1 bash run_m6_sitl.sh 70        # SITL：双机 READY 后跑 live+logs，不通过则放弃起飞
PREFLIGHT_GPS=1 PREFLIGHT=1 bash run_m6_sitl.sh 70   # 再加 GPS 严格门
PREFLIGHT_PARAMS=1 PREFLIGHT=1 bash run_m6_sitl.sh 70 # 再加飞控参数体检（MAVLink 18570/18571）
bash run_m6_real.sh                        # 真机：默认跑 --live --gps；PARAM_MAVLINK 时跑参数体检
```

**companion 安全网**（对标参考 `safety_filter.py`；`report/companion_safety.md`）：
```bash
# 失联看门狗：丢 A 状态 → 就地冻结 → AUTO.LAND（飞行中 kill A 节点可诱发）
LAUNCH_EXTRA="peer_loss_hold_s:=1.0 peer_loss_land_s:=4.0" bash run_m6_sitl.sh 55
# 硬碰撞地板：太近 → 去掉朝 A 分量 + 持续→HOLD（抬阈值可诱发）
LAUNCH_EXTRA="collide_warn:=1.7 collide_emerg:=1.5" bash run_m6_sitl.sh 45
```

**安全层故障注入矩阵**（对标参考 S27–S33；每道保护单独诱发 + 零误触发基线）：
```bash
python3 tools/sitl_safety_matrix.py            # 5 场景 ~10min，PASS/FAIL → report/safety_injection_matrix.md
python3 tools/sitl_safety_matrix.py --only collision_floor_hold
```

---

## 1. 离线单元自测（不需要 ROS / SITL）

### 1.1 一键自检（推荐，改代码后必跑）

```bash
make check-quick                   # = bash tools/run_checks.sh --quick（秒级）
make check                         # = bash tools/run_checks.sh（全量）
bash tools/run_checks.sh --quick   # 秒级：纯模块自测 + tools/test_*.py（pre-commit）
bash tools/run_checks.sh           # 全量：再加 acados 链 + offline_run.py --all + pytest
```

> `run_checks.sh` 会**自动 `source env.sh`**（存在时），无需手动 export acados 路径；
> 失败时退出码非 0 并列出失败项。风格参考 `~/drone_package_20260908` 的 `*_check.sh`。
> 可选装提交钩子：`git config core.hooksPath .githooks`（提交前跑 `--quick`）。

### 1.2 逐模块自测

```bash
python3 -m payload_catch.payload_model   # 载荷抛体模型（解析 vs 积分）
python3 -m payload_catch.rendezvous      # 会合规划
python3 -m payload_catch.sim_core        # 离线闭环仿真
python3 -m payload_catch.payload_filter  # KF / 增广风 KF 估计
python3 -m payload_catch.stack_drop      # M6 垂直堆叠解析规划
python3 -m payload_catch.relnav          # 相对定位纯逻辑（大地→NED/杆臂/相对化）
python3 -m payload_catch.contact_detect  # 接触检测
python3 -m payload_catch.uncertainty     # 相对不确定度模型
python3 -m payload_catch.dynamics        # 四旋翼倾角/推力约束
python3 -m payload_catch.impact          # 接触冲击/可恢复性
python3 -m payload_catch.perception      # 视觉感知模型
python3 -m payload_catch.stats           # Wilson CI / McNemar
python3 -m payload_catch.safety_logic    # 安全层纯逻辑（围栏/姿态/看门狗/状态机）
```

### 1.3 `tools/test_*.py`（纯逻辑单元测试，✅/❌ + 退出码）

```bash
python3 tools/test_coord_cert.py     # C1 概率证书（Rice/阈值/单调性/MC 校核）
python3 tools/test_keepout.py        # C5 handover-CBF（投影可行性 + 不变性）
python3 tools/test_safety_logic.py   # 安全状态机/围栏/姿态滤波/传感器看门狗
python3 tools/test_config.py         # yaml 单一真值源契约（layout 引用/几何/真机段）
python3 tools/test_purity.py         # 纯算法层“无 ROS 依赖”守卫
python3 tools/test_compileall.py     # 全仓 .py 语法编译守卫（含 ROS 节点/launch）
python3 tools/test_preflight.py      # 起飞前自检逻辑
python3 tools/test_live_probe.py     # 运行时探测判定
python3 tools/test_uplink_test.py    # 上行链路判定
python3 tools/test_telemetry.py      # 遥测有效性（STALE/OVERFLOW/SILENCE/HOVER）

python3 -m pytest -q                 # 收进标准测试框架（与上面同一批脚本）
```

> **CI**：`.github/workflows/checks.yml` 在 push/PR 时跑 `run_checks.sh --quick` + `pytest`
> （纯 Python，不需要 ROS/acados）。SITL/acados 检查在本地全量模式跑。
>
> **SITL 端到端验收**（需完整 SITL 环境，耗时数分钟，不进 CI）：
> ```bash
> source env.sh && bash tools/sitl_check.sh 70    # STACK CAPTURED 且无 failsafe ⇒ 退出 0
> ```

### 1.4 acados 链冒烟

```bash
source env.sh && python3 tools/smoke_acados.py   # codegen+编译+求解+50 次耗时
python3 -m payload_catch.mpc_terminal            # MPC 求解器自测（需 acados）
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

# 真机末端：圆形托盘（塑料围边 + 泡棉缓冲）；载荷 6cm / 100g 方块
python3 tools/stack_run.py --scenario M6_stack_tray --mc 300        # 内径30cm：99% (296/300)
python3 tools/stack_run.py --scenario M6_stack_tray_small --mc 300  # 内径25cm：93%
python3 tools/stack_run.py --scenario M6_stack_tray_wind --mc 300   # +侧风3m/s：96%
python3 tools/tray_sizing.py                    # 托盘选型：内径/围边/泡棉e/gap（默认30cm/5cm/0.15）
python3 tools/tray_sizing.py --e 0.7            # 看裸塑料盘会怎样（FAIL：回弹49cm）
python3 tools/tray_sizing.py --sweep-e          # e → 所需围边高度表（选泡棉用）
python3 tools/tray_sizing.py --measure-drop 1.0 0.22   # 落物试验反推恢复系数 e
```

> **圆形托盘物理**：托盘没有杯深，靠**围边挡横向 + 泡棉消反弹**。保持条件
> `回弹高度 e²·gap ≤ 有效围挡高度 h（围边+凹垫）`；落点半径 `eff_r = 盘内半径 − 物半宽`。
> **恢复系数 `e` 是生死线**：裸塑料 `e≈0.7` → 回弹 ~20cm 必飞（0%）；软泡棉 `e≤0.15` → 回弹 ~2cm
> （99%）。真机务必先用 `--measure-drop` 实测泡棉的 `e`，要求 ≤ 0.20。

---

## 3b. 生成 Word 仿真报告（`tools/make_docx_report.py`）

> 需 `python-docx`（`pip install --break-system-packages python-docx`）。
> 报告内容：相关研究（引言）/ 理论分析 / 仿真条件 / 仿真数据（含状态数据）/ 仿真分析。
> 数据为**实时运行** M1–M4 离线 + M6 蒙特卡洛采集，保证与当前代码一致。

```bash
cd ~/drone_payload_catch
export ACADOS_SOURCE_DIR=/home/caolihao/drone_package_20260908/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
python3 tools/make_docx_report.py                 # → report/drone_payload_catch_sim_report.docx
python3 tools/make_docx_report.py --fast          # 缩小 MC 次数，快速预览
python3 tools/make_docx_report.py --out /tmp/x.docx
```

> Word 中打开后按 **F9** 更新目录/页码。

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
| `FUNNEL_TYPE` | `flat` | `flat`=实心盘；`cup`=空心导向锥杯 `x500_funnel_cup`（§4.1d）；`tray`=**圆形托盘**（围边+泡棉，§4.1e） |
| `TRAY_RIM` | `0.05` | `tray` 围边高（m，=有效围挡高度，决定 `v_retain`） |
| `TRAY_E` | `0.15` | `tray` 泡棉恢复系数（低回弹；v_retain=√(2gh)/e） |
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

### 4.1e 圆形托盘（真机末端，新增 ✅）

把末端换成**塑料圆形托盘（围边 + 泡棉缓冲）**，载荷用 **6cm / 100g 方块**（`models/payload_100g`）。
物理保持靠"围边挡横向 + 泡棉消反弹"（`h ≥ e²·gap`）；落点半径 `eff_r = 盘内半径 − 物半宽`；
保持速度 `v_retain = √(2·g·h)/e`（h=围边高）。

```bash
cd ~/drone_payload_catch
FUNNEL_TYPE=tray bash run_m6_sitl.sh 60      # 无窗口（内径30cm/围边5cm/泡棉e=0.15）
FUNNEL_TYPE=tray bash run_m6_gui.sh 110      # GUI
# 改盘径/围边/泡棉：
FUNNEL_TYPE=tray FUNNEL_MOUTH=0.125 TRAY_RIM=0.06 TRAY_E=0.12 bash run_m6_sitl.sh 60
```

模型由 `python3 tools/gen_tray.py` 生成（`models/x500_tray` = x500 + 托盘；独立 `models/funnel_tray`）。

**实测（`~/payload_catch_m6/launch.log`，3/3 成功）**：
```
end-effector: type=tray mouth(盘内半径)=0.15 eff=0.12 v_retain=6.6
B: DIVE plan a_dive=0.00(auto=True) t_c≈0.40s v_rel≈3.9 v_retain=6.603 feasible=True
*** STACK CAPTURED *** horiz=0.015–0.049m rel_v≈2.7–2.8m/s
B phase=LAND caught=True （载荷随托盘携带到落点）
px4_0/px4_1: 无 Failsafe；min|A−B|≥1.08m
```
离线对应工况 `M6_stack_tray*`（见 §3）：`python3 tools/stack_run.py --scenario M6_stack_tray --mc 300`。

**大托盘（内径40cm，eff_r=0.17）**：`FUNNEL_MOUTH=0.20` 自动选用 `models/x500_tray_big`
（由 `TRAY_R_IN=0.20 ... python3 tools/gen_tray.py` 生成）。用于**运动交接**（见下）。

**编队同速投放 × 托盘**（`FORMATION_VEL` 非零 + `FUNNEL_TYPE=tray`，载荷自动切 100g 挂载型）：
```bash
FORMATION_VEL="0.5,0.0,0.0" FUNNEL_TYPE=tray FUNNEL_MOUTH=0.20 bash run_m6_sitl.sh 75
FORMATION_VEL="1.0,0.0,0.0" FUNNEL_TYPE=tray FUNNEL_MOUTH=0.20 bash run_m6_sitl.sh 75
```
实测（大托盘 40cm）：0.5 与 1.0 m/s 均 `STACK CAPTURED`（horiz 0.065–0.108m）→ 载荷随托盘携带落地；
小托盘(30cm)在运动中会被“滑出”（软件仍报 captured）→ 运动交接对盘径要求高于定点。详见
`report/m6_tray_robustness.md` §5。

**主动锁扣（让运动交接与盘径无关）**：`FUNNEL_TYPE=tray PAYLOAD_LOCK=1`（自动用 `payload_lock_100g`）——
捕获时在 B 末端就地重生成并建固定关节。实测小托盘(30cm) 定点/编队0.5/编队1.0 均**捕获→锁死→带走**：
```bash
FUNNEL_TYPE=tray PAYLOAD_LOCK=1 bash run_m6_sitl.sh 70
FORMATION_VEL="0.5,0.0,0.0" FUNNEL_TYPE=tray PAYLOAD_LOCK=1 bash run_m6_sitl.sh 75
FORMATION_VEL="1.0,0.0,0.0" FUNNEL_TYPE=tray PAYLOAD_LOCK=1 bash run_m6_sitl.sh 75
```

**泡棉恢复系数 e 实测（真机必做）**：
```bash
# 真机测量：从高度 H 落 6cm/100g 方块，量回弹 h → e=√(h/H)
python3 tools/foam_drop_test.py --h-drop 1.0 --rebounds 0.020 0.024 0.018
# 虚拟落物台（验证方法 + 模型参考；模型泡棉 e≈0）
bash tools/gz_foam_drop_test.sh
```

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

# 圆形托盘（真机末端）扫描 / 蒙特卡洛 → report/m6_tray_sitl_results.md / m6_tray_sitl_mc.md
bash tools/sweep_m6_tray_sitl.sh            # 8 档难度扫描（FUNNEL_TYPE=tray）
NREP=3 bash tools/mc_m6_tray_sitl.sh        # 蒙特卡洛（N=3/档）

# 托盘倾斜鲁棒性：倾斜落物台（gz 物理，秒级）→ report/m6_tray_robustness.md
bash tools/tray_tilt_test.sh                # 托盘 vs 空心杯 vs 平盘，扫倾角 0–50°
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

---

## 8. 真机接入（Bring-up，无 Gazebo/PX4）

> 完整清单见 **`report/real_hardware_bringup.md`**。此处只列命令。

```bash
# 相对定位驱动：把两机 RTK 换算成 /drone_a/state（替换仿真真值替身）
#   rtk_type: navsatfix（sensor_msgs/NavSatFix）或 array（Float64MultiArray [lat,lon,alt] 或 [n,e,d]）
ros2 run payload_catch relnav_node --ros-args \
  -p source:=rtk -p a_rtk_topic:=/rtk/a -p b_rtk_topic:=/rtk/b \
  -p rtk_type:=navsatfix -p use_geodetic:=true \
  -p a_lever:="[0.0,0.0,0.2]" -p b_lever:="[0.0,0.0,-0.21]"

# 相对定位纯逻辑自测 + 接触检测自测
python3 -m payload_catch.relnav
python3 -m payload_catch.contact_detect

# 泡棉恢复系数 e 实测换算（真机落物台）
python3 tools/foam_drop_test.py --h-drop 1.0 --rebounds 0.020 0.024 0.018

# 真机任务启动（无 Gazebo/PX4；A 不广播真值，改由 relnav 发布；接触检测触发捕获）
A_RTK=/rtk/a B_RTK=/rtk/b B_OFFSET="5.0,0.0,0.0" bash run_m6_real.sh
# 或直接：
ros2 launch payload_catch catch_real_launch.py \
  a_hover:="[0,0,-4.5]" b_standby:="[0,0,-3.5]" b_offset:="[5.0,0,0]" \
  source:=rtk a_rtk_topic:=/rtk/a b_rtk_topic:=/rtk/b \
  a_lever:="[0,0,0.2]" b_lever:="[0,0,-0.21]" contact_detect:=true

# SITL 里验证接触检测（默认关，开启后以接触事件触发捕获）
FUNNEL_TYPE=tray LAUNCH_EXTRA="contact_detect:=true" bash run_m6_sitl.sh 60
```

**关键前置**：① RTK 相对定位精度 σ ≪ `eff_r`（0.12–0.17m，建议 ≤0.03m）；② 载荷 `/payload/state`
由真实感知（视觉/动捕/UWB tag 或弹道预测）发布；③ 杆臂/原点/几何**标定回填** `config` 的 `real:` 段；
④ 泡棉 e 实测（≤0.20）；⑤ 安全员 + RC 接管 + 围栏。

### 8.1 相对不确定度修正（修"绝对 σ 当相对 σ"）

```bash
python3 -m payload_catch.uncertainty     # 相对不确定度模型自测
python3 tools/rel_sigma.py               # 量化学 → report/rel_uncertainty.md

# SITL：证书闸在修正 σ 后可放行（相对传感器架构 σ=0.03）
COORD=handshake FUNNEL_MOUTH=0.20 FUNNEL_TYPE=tray \
  LAUNCH_EXTRA="sigma_model:=relative sigma_sensor:=0.03 gate_use_relative:=true release_gate_mode:=certificate" \
  bash run_m6_sitl.sh 50

# 绝对广播架构 + 公共抵消（σ_A=0.15, ρ=0.95）
COORD=handshake FUNNEL_MOUTH=0.20 FUNNEL_TYPE=tray \
  LAUNCH_EXTRA="sigma_model:=relative sigma_a:=0.15 sigma_rho:=0.95 gate_use_relative:=true release_gate_mode:=certificate" \
  bash run_m6_sitl.sh 50
```

关键旋钮：`sigma_model=legacy|relative`；`sigma_sensor`（相对传感器 σ，>0 时忽略绝对 eph）；
`sigma_a`/`sigma_rho`（绝对广播架构的 A 误差与公共相关系数）；`lever_a/b`、`sigma_att_a/b`（杆臂×姿态）；
`gate_use_relative`（A 启发式闸用相对 σ，不再叠加绝对 eph）。

### 8.2 动力学限幅 + 接触冲击（控制/安全层加强）

```bash
python3 -m payload_catch.dynamics        # 四旋翼倾角+推力聚合约束（自测）
python3 -m payload_catch.impact          # 接触冲击/可恢复性/带载推力余量（自测）
python3 tools/dynamics_contact.py        # 量化学 → report/dynamics_contact.md

# 接空安全中止（DIVE 超时未捕获 → MISS，安全悬停→降落）
FUNNEL_TYPE=tray LAUNCH_EXTRA="release_xy_sigma:=3.0 track_payload:=false miss_timeout_s:=2.0" \
  bash run_m6_sitl.sh 45
```

关键旋钮：`miss_timeout_s`（默认 4s；超时未捕获 → 发 `/payload/miss` 并安全降落，不盲目追击/砸地）。
`dynamics.py` 给出 `a_x ≤ (g−a_z)·tanθ_max`（水平权限随下潜衰减）与推力约束；
`impact.py` 给出冲击可恢复条件 `ω=J·d_off/I_B ≤ τ_max·t/I_B` 与带载悬停条件 `T_max ≥ (m_B+m_p)g`。

### 8.3 感知 / 接触 / 风 保真（补充）

```bash
# 视觉载荷感知模型（相机：FOV/距离相关误差/深度/丢帧）
python3 -m payload_catch.perception            # 自测
python3 tools/perception_study.py              # 相机 vs 真值替身 → report/perception_study.md
python3 tools/stack_run.py --scenario M6_stack_tray_cam --mc 100   # 相机感知工况

# 柔性接触（弹簧-阻尼：把峰值力降一个量级）
python3 -m payload_catch.impact                # 自测（含 complanit_contact）
python3 tools/dynamics_contact.py              # 含 §3b 柔性接触表

# SITL 风场（载荷 enable_wind=true，注入 gz WindEffects 插件）
WIND=6.0 WIND_DIR=0 FUNNEL_TYPE=tray FUNNEL_MOUTH=0.20 bash run_m6_sitl.sh 60
WIND=6.0 FUNNEL_TYPE=tray bash run_m6_gui.sh 110     # GUI
# 或先生成带风 world 再跑：
python3 tools/make_wind_world.py --src "$SITL_WORLD" --out /tmp/default_wind.sdf --wind 6 --dir-deg 0
SITL_WORLD=/tmp/default_wind.sdf bash run_m6_sitl.sh 60
```

要点：相机误差**距离相关**（近场好、远场差、有 FOV/丢帧）——好相机近场优于固定 σ=0.05 替身，
差相机（窄 FOV/高丢帧）在大释放误差/编队下大幅退化；柔性接触峰值力 ~30× 降低（需行程在围边内）；
SITL 风对 100g 致密方块漂移很小（与离线一致）。

### 8.4 统计 + 基线（离线 · Wilson CI · 配对检验）

```bash
python3 -m payload_catch.stats                 # 统计工具自测（Wilson CI / McNemar）
python3 tools/stats_report.py --n 2000 --rv-n 30   # → report/statistics.md
```

输出：M6 末端机构/感知成功率 + **Wilson 95% CI**；感知、预测对正、闭环重规划消融的
**配对 McNemar** p 值；释放误差敏感性；参数不确定性。SITL 蒙特卡洛脚本
（`mc_m6_sitl.sh` / `mc_m6_tray_sitl.sh`）也已附 Wilson CI。
