# 代码与数据流地图（references）

## 算法层（纯 Python，无 ROS 依赖）

| 模块 | 作用 |
|---|---|
| `payload_catch/payload_model.py` | 载荷抛体（解析；可选阻力/风） |
| `payload_catch/rendezvous.py` | 会合规划（min-energy 三次 + 软终端速度 + solve_inflight） |
| `payload_catch/sim_core.py` | 离线闭环仿真（M1–M4） |
| `payload_catch/stack_drop.py` | M6 垂直投放解析规划 + 漏斗/托盘判据 + 离线仿真 |
| `payload_catch/mpc_terminal.py` | acados 终端 MPC |
| `payload_catch/payload_filter.py` | 载荷状态估计（KF/朴素） |
| `payload_catch/uncertainty.py` | 相对不确定度模型 |
| `payload_catch/dynamics.py` | 四旋翼倾角+推力聚合约束 |
| `payload_catch/impact.py` | 接触冲击/可恢复性/带载余量 |
| `payload_catch/coord_cert.py` | C1 捕获概率证书（Rice/Marcum-Q） |
| `payload_catch/relnav.py` | 相对定位纯逻辑（大地→NED、杆臂、原点无关相对化） |
| `payload_catch/contact_detect.py` | 接触检测（加速度尖峰/速度反转/外部开关） |
| `payload_catch/keepout.py` | C5 handover-CBF（速度级防碰投影 + 不变集） |
| `payload_catch/stats.py` | 统计（Wilson CI + 配对 McNemar） |
| `payload_catch/safety_logic.py` | 安全层**纯函数**（软围栏/姿态滤波/EKF 看门狗/分级状态机；从 `px4_iface` 抽出以便离线单测） |
| `payload_catch/telemetry.py` | **遥测有效性**纯逻辑（冻结 STALE / 溢出 OVERFLOW / 沉默 SILENCE / 兜底 HOVER） |

## ROS 节点（SITL/真机）

| 节点 | 说明 |
|---|---|
| `a_node.py` | A：起飞→悬停/编队→释放；广播 `/drone_a/state`（或由 relnav 替换） |
| `b_node.py` | B：会合捕获 或 M6 状态机（CLIMB→WAIT_A→TRANSLATE→ALIGN→DIVE→DONE→LAND） |
| `payload_node.py` | 载荷：Gazebo 生成/瞬移/状态发布；锁扣/挂载 |
| `relnav_node.py` | 真机相对定位驱动（RTK/px4/sim → `/drone_a/state`） |
| `px4_iface.py` | PX4 接口基类（话题/QoS/ARM+OFFBOARD/安全层） |

## 关键话题

- `/drone_a/state`：`[t, pA_world_NED(3), vA_world_NED(3)]`
- `/payload/state`：`[t, p(3), v(3)]`
- `/payload/caught`、`/payload/miss`、`/payload/contact`、`/payload/release_at`
- `/drone_b/ready`：`[ready, rel_xy, spd_xy, stamp, sigma_abs, sigma_rel]`
- `/safety/kill_a|b`

## CLI 工具（`tools/`）

| 工具 | 作用 |
|---|---|
| `offline_run.py` | M1–M4 离线体检（`--all` / `--plot` / `--mc`） |
| `stack_run.py` | M6 离线体检查询（`--mc` / `--sweep-*`） |
| `tray_sizing.py` | 圆形托盘选型 |
| `foam_drop_test.py` | 泡棉 e 换算 |
| `gz_foam_drop_test.sh` | 虚拟落物台测 e |
| `tray_tilt_test.sh` / `funnel_drop_test.sh` | 倾斜保持物理测试 |
| `gen_tray.py` | 生成托盘模型 |
| `rel_sigma.py` | 相对不确定度量化 |
| `dynamics_contact.py` | 动力学+接触量化 |
| `make_docx_report.py` | Word 仿真报告 |
| `run_checks.sh` | **一键离线自检**（`--quick`/全量；纯模块 + `test_*.py` + acados + `offline_run --all`） |
| `test_coord_cert.py` / `test_keepout.py` / `test_safety_logic.py` | **纯逻辑单元测试**（C1 证书 / C5 CBF / 安全层） |
| `test_config.py` / `test_purity.py` / `test_compileall.py` | 契约守卫（yaml 单一真值源 / 纯算法层不得依赖 ROS / 全仓语法） |
| `smoke_acados.py` | acados MPC 链冒烟 |
| `sitl_check.sh` | M6 SITL 端到端验收（委托 `run_m6_sitl.sh`，按事件给退出码） |
| `preflight_check.py` | **起飞前自检**（离线 / `--sitl` / `--live` / `--gps` / `--logs`） |
| `live_probe.py` | **运行时持续探测**（EKF/failsafe/磁罗盘/IMU/GPS；纯逻辑 `evaluate`） |
| `uplink_test.py` | **offboard 上行链路验证**（发心跳看 `offboard_control_signal_lost`） |
| `check_log_validity.py` | **遥测有效性检查**（launch.log 的 STALE/OVERFLOW/SILENCE/HOVER） |
| `pytest.ini` + `tests/` | 把 `tools/test_*.py` 收进 `pytest -q`（同步 `.github/workflows/checks.yml` CI） |

## SITL 脚本（无窗口 / GUI）

- `run_m1_sitl.sh` / `run_m1_gui.sh`：M1 水平会合。
- `run_m6_sitl.sh` / `run_m6_gui.sh`：M6（`FUNNEL_TYPE=flat|cup|tray`，`FUNNEL_MOUTH`，`FORMATION_VEL`，`PAYLOAD_LOCK`）。
- `run_m6_real.sh` + `launch/catch_real_launch.py`：真机（无 Gazebo）。

## 配置

`config/catch_scenarios.yaml`：`defaults` / `layouts` / `scenarios` / `thresholds` / `real`。
