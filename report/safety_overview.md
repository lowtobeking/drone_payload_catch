# 安全总览（飞行前 · 飞行中 · 验证）

> 集中回答"现在的安全做到哪了、和参考工程 `drone_package_20260908` 比如何、还缺什么"。
> 分报告：`fcu_safety.md`（飞控参数+电池）、`companion_safety.md`（失联+碰撞地板）、
> `safety_supervisor.md`（kill）、`safety_control_review.md`（旧缺口清单）、
> `safety_injection_matrix.md`（故障注入结果）、`opt_round2..5.md`（分级/传感器/姿态）。

---

## 1. 飞行前检查

| 层 | 工具 | 检查 |
|---|---|---|
| 静态环境 | `tools/preflight_check.py`（`--sitl` 严格） | 配置/依赖/`PX4_DIR`+SITL bin/`SITL_WORLD`/`RMW=fastrtps`/`GZ_RESOURCE`/`acados`/`gz·ros2·agent` |
| 运行时 `--live` | `tools/live_probe.py`（**持续订阅**取聚合） | EKF `xy/z/v_valid` 比例、非 `dead_reckoning`、eph/epv；`failsafe_flags`；`estimator_status_flags`（tilt/yaw、磁在线/故障、加计故障）；`battery_status` |
| 日志门 `--logs` | `preflight_check.py` | 双机 `Ready for takeoff`；`Gyro STALE`/`Arming denied` 致命；其余 `Preflight Fail` 分级 warn |
| 飞控参数 `--params` | `tools/preflight_params.py`（`--file`/`--mavlink`） | `MPC_THR_MIN<MPC_THR_HOVER`、`COM_RCL_EXCEPT` 不含 bit2、`RTL_RETURN_ALT≤GF_MAX_VER_DIST`、电池阈值次序、失效保护=Land、围栏启用、EKF 源一致 |
| GPS（室外） | `preflight_check.py --gps` | 持续 `eph≤1.5 / epv≤2.5 / sats≥20` |
| 上行链路 | `tools/uplink_test.py` | 发 offboard 心跳看 `offboard_control_signal_lost` 翻转 |
| 遥测有效性 | `tools/check_log_validity.py` | `STALE/OVERFLOW/SILENCE/HOVER`（飞后/飞前日志） |

**门禁**：SITL `PREFLIGHT=1 [PREFLIGHT_GPS=1] [PREFLIGHT_PARAMS=1] bash run_m6_sitl.sh 70`；
真机 `run_m6_real.sh` 默认 `--live --gps`，`PARAM_MAVLINK` 时跑参数体检。失败即拒飞。

## 2. 飞行中安全

| 机制 | 实现 | 默认 |
|---|---|---|
| offboard 心跳丢失 | PX4 自带 failsafe + 参数体检保证 `COM_OF_LOSS_T/COM_OBL_RC_ACT` | 开 |
| 分级安全状态机 | `px4_iface`+`safety_logic`：`OK/HOLD/PULLBACK/LAND/KILL`（姿态/越界/状态超时/传感器/低电/外部原因） | 开（`auto_kill=false`） |
| 软围栏 + 越界回拉 | `_fence_velocity`（保留切向，不顶任务） | 开 |
| 反应式 keep-out + CBF | `b_node`（`min_ab_gap+kσ`；`keepout_mode=cbf`） | 开（heuristic；CBF opt-in） |
| **硬碰撞地板** | `keepout.hard_floor` + `external_safety_reasons` → HOLD | 开（阈值低于正常间距） |
| 姿态/角速率/指令加速度约束 | `_attitude_govern` + `sp_rate_limit` | 开 |
| 传感器/估计器约束 | eph/epv σ、健康看门狗、`dead_reckoning`、estimator reset、估计器限值 | 开 |
| 释放证书闸 + 握手 + 单次释放 | `a_node`/`coord_cert` | 开 |
| 释放超时 abort / 接空 MISS→悬停→降落 | `b_node` | 开 |
| **失联看门狗** | `safety_logic.peer_loss_action` + `b_node`：丢 A→冻结→AUTO.LAND | 0=关（真机开） |
| **电池低电→Land** | `safety_logic.battery_*` + `px4_iface` | 开 |
| 飞行终止 kill | `/safety/kill_a|b` + 异常自动 | 手动/opt-in |
| 任务完成/捕获后自动降落 | `b_node` 各自 land_xy → AUTO.LAND | 开 |

## 3. 验证

| 类别 | 手段 | 结果 |
|---|---|---|
| 离线自检 | `tools/run_checks.sh`（纯模块+`test_*`+acados+offline+pytest） | ✅ 32/32；pytest 13 |
| CI | `.github/workflows/checks.yml`（push/PR，quick+pytest） | 已加（待首个 PR 验证） |
| SITL 起飞前门 | `PREFLIGHT=1`（live+logs+battery）+ `PREFLIGHT_PARAMS=1`（两台各读 1016 参数） | ✅ fail=0 |
| 端到端 | `tools/sitl_check.sh` | `STACK CAPTURED`、无 failsafe |
| **故障注入矩阵** | `tools/sitl_safety_matrix.py`（5 场景） | ✅ **5/5 PASS**（`safety_injection_matrix.md`） |
| 纯逻辑单测 | `test_{safety_logic,keepout,live_probe,preflight,fcu_params,safety_matrix,telemetry,...}` | ✅ |

## 4. 与参考工程 `drone_package_20260908` 对照

| 维度 | 参考 | 本工程 | 判定 |
|---|---|---|---|
| 飞控参数体检 | `fc_configure.py` 分组 | `fcu_params`+`preflight_params`+`fcu_configure` | ✅ 相当 |
| 电池低电保护 | FC `COM_LOW_BAT_ACT=Land` | 飞行中 warning≥2/<0.07→LAND + 起飞前查 | ✅ 相当 |
| 飞散/高度围栏 | track-divergence 刹停 + FC `GF_*` | `_fence_velocity` 回拉 + 参数校验 | ✅ 相当 |
| 硬碰撞地板 | `safety_filter` 缺口 2（实测邻居） | `keepout.hard_floor`+HOLD | ✅ 相当 |
| 失联自动降落 | 三层（FC offboard/RC/leader 丢失） | FC 层 + `peer_loss` 冻结→LAND | ✅ 相当 |
| 估计健康门 | `est_ok`→HOLD→RELINQUISH | eph/epv/dead_reckoning/reset→HOLD | ✅ 相当 |
| 姿态/加加速度限幅 | max_speed/climb/accel+jerk | `_attitude_govern`+`sp_rate_limit`（无显式 jerk） | 🟡 略逊 |
| 故障注入验证 | S27–S33 清单 | `sitl_safety_matrix.py` 5 场景 | ✅ 相当 |
| 形式化保证（证书/CBF） | 无 | C1 证书 + C5 CBF | ✅ **更强** |
| 可测性/CI | 纯函数+自测 | 纯函数+32 项+pytest+CI | ✅ 更强 |
| 真机 / HIL | 有（首飞/日志/标定） | 无 | ❌ 落后 |
| 多机规模 | 1→9 机编队 | 2 机 A/B | ❌ 落后 |
| `DEGRADED`/`RELINQUISH` 语义 | DEGRADED 半速 + 交还 PX4 | HOLD/LAND/KILL（HOLD 仍发心跳、不交还） | 🟡 设计取舍，已文档化 |

## 5. 仍缺（诚实）

1. **真机 / HIL 验证**（硬门槛；本工程仅 SITL）。
2. **多机规模 / 跨机冗余**（单估计器、单链路）。
3. **显式 jerk 硬帽**（现有指令加速度限额，无 jerk）。
4. **接空失败载荷砸地物理防护/告警**（仅记录 MISS）。
5. `RELINQUISH`（停流交还 PX4）语义未做——我们以 HOLD/LAND/KILL 替代，`HOLD` 期间仍发 offboard 心跳（参考工程指出这会阻止 PX4 接管，是刻意取舍）。

## 6. 一键复现

```bash
source env.sh
bash tools/run_checks.sh                         # 离线 32 项 + pytest
PREFLIGHT=1 PREFLIGHT_PARAMS=1 bash run_m6_sitl.sh 70   # 起飞前全门
python3 tools/sitl_safety_matrix.py              # 故障注入矩阵 5 场景
```
