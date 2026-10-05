# 优化第四轮：传感器/估计器约束（用已有 PX4 字段，不加硬件）

> 承接 `report/opt_round2.md` / `opt_round3.md`。本轮把**已经订阅的 `vehicle_local_position` 里
> 未被使用的 EKF 派生量**接成真正的约束：不确定度、健康/一致性、估计器限值。
> 原则：**不加新传感器、默认行为兼容、可逐项开关**。
>
> 进度：**① EKF σ ✅ · ② 健康看门狗 ✅ · ③ 估计器限值 ✅**

---

## 背景：能用的"传感器"其实已经在消息里

`px4_iface._on_lpos` 原先只取了 `x,y,z,vx,vy,vz,xy_valid,z_valid`。同一条消息里还有：
`eph/epv/evh/evv`（位置/速度 σ）、`dead_reckoning`、`heading_good_for_control`、
`*_reset_counter`、`vxy_max/vz_max/hagl_min/...`（估计器限值）。
这些全是 PX4 EKF（IMU+磁力计+气压计+GNSS）融合出来的，**零新硬件**。

---

## ① 不确定度约束：用 `eph/epv` 当 σ ✅

- `px4_iface` 暴露 `pos_sigma_h = eph`、`pos_sigma_v = epv`（`sensor_use_ekf_sigma` 控制）。
- **B 的 keep-out**：`_gap_eff = min_ab_gap + k·max(σ_est, epv)`。
- **B 上报给 A 的 σ**：`σ = √(σ_est² + eph²)`。
- **A 的释放余量闸**：`σ = min(max(release_sigma, b_sigma, eph_A), release_sigma_max)`。

这比 A2 原计划"自写相对位置 KF"更直接：PX4 已给出标定过的协方差。

## ② 健康/一致性看门狗 ✅

`_sensor_health_reasons()` 返回异常，喂进第二轮的**分级安全状态机**（可恢复 → HOLD）：

| 检查 | 字段 | 默认 |
|---|---|---|
| 位置/速度有效性 | `xy/z/v_xy/v_z_valid` | 开 |
| 航位推算 | `dead_reckoning` | 开 |
| 位置 σ 超限 | `eph/epv > sensor_eph_max/epv_max` | 开 |
| 估计器跳变 | `*_reset_counter` 变化后保持 `sensor_reset_hold_s` | 开 |
| 航向可用性 | `heading_good_for_control` | **默认关**（本仿真常 false，见下） |

## ③ 估计器限值约束 ✅

- `publish_velocity` 里把水平/垂直指令限制到 `vxy_max / vz_max`（`inf` 自动忽略）。
- `_fence_velocity` / 安全状态机的alt 下限并入 `hagl_min`（**仅在有限时**；无测距/光流时为 `inf`，忽略）。

---

## 🩸 踩坑记录（重要）

1. **`hagl_min=inf` 不能映射为 0**：无测距/光流时 `get_ekf_ctrl_limits` 全部返回 `NAN`，EKF2 转 `INFINITY`。
   我最初把 `inf→0` 当高度下限，导致 `alt_floor` 从 `-1.0` 抬到 `0.0`，地面微负高度立刻触发 geofence→HOLD，起飞失败。
   **修正**：`_est_hagl_min` 保留 `inf`，只在 `math.isfinite` 时才并入下限。
2. **`heading_good_for_control` 在本仿真长期 false**：它等价 `isYawFinalAlignComplete()`；本机 x500 磁罗盘
   本就有已知健康告警（参考 MEMORY §7.6）。把它当硬约束会**永久 HOLD**。
   **修正**：`sensor_watchdog_heading` 默认 `false`（需要时再开）。
3. **估计器限值在 x500 上全是 `inf`**（无测距/光流），所以 ③ 在标称下是 no-op——这是**预期**，字段留给装了测距/光流的机型。

---

## 新增参数（launch 可覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `sensor_constraints_enable` | `true` | 总开关 |
| `sensor_use_ekf_sigma` | `true` | 用 `eph/epv` 作 σ |
| `sensor_watchdog_enable` | `true` | 健康/一致性看门狗 |
| `sensor_use_est_limits` | `true` | 用估计器限值 |
| `sensor_eph_max` / `sensor_epv_max` | `0.50 / 0.50` | 位置 σ 上限 (m) |
| `sensor_reset_hold_s` | `1.0` | 估计器跳变后保持 HOLD 多久 (s) |
| `sensor_watchdog_heading` | `false` | 航向可用性也当硬约束 |

---

## 验证（2026-10-05）

单元（构造 `VehicleLocalPosition`）：
```
healthy          → []（无异常）；hagl_min=inf 被忽略
dead_reckoning   → ['dead_reckoning']
eph=1.0          → ['eph=1.00>0.50']
reset counter +1 → ['estimator_reset']
估计器限值 vxy=1.0/vz=0.5：[3,0,2] → [1,0,0.5]
```
SITL（标称）：
```
[a] SAFETY HOLD: estimator_reset → 1s 后 恢复 OK（每机各 1 次，仅启动期）
*** STACK CAPTURED horiz=0.034m ，双机落地、无 failsafe
safe 统计：OK 56 次 / HOLD 2 次
```
说明：看门狗确实捕获到了启动期 EKF reset 并短暂 HOLD，随后自愈，未影响任务——正是设计意图。

改动文件：`payload_catch/px4_iface.py`、`b_node.py`、`a_node.py`、`launch/catch_stack_launch.py`。
