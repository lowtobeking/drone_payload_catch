# 优化第五轮：姿态/角速率约束（IMU → 安全滤波）

> 承接 `report/opt_round4.md`。目标：把**已知的飞控姿态 failsafe 边界**（MEMORY 记录 `v_p≈4.9 m/s`
> 触发 `Attitude failure (roll)`）变成显式约束——用 IMU 派生的倾角/角速率**主动限制水平速度指令**，
> 而不是等 PX4 自己终止。
>
> 原则：全部来自**已订阅**的 `vehicle_attitude` / `vehicle_local_position`，零新话题/零新硬件；
> 只在接近姿态极限时介入，标称不受影响。

---

## 1. 为什么不用 `vehicle_angular_velocity`

`VehicleAngularVelocity` 消息存在，但在 PX4-1.16 的 `dds_topics.yaml` 里
`/fmu/out/vehicle_angular_velocity` **被注释掉（默认不发布）**。所以角速率改为：
**对已订阅的 `vehicle_attitude` 四元数解出的 roll/pitch/yaw 做时间有限差分**（含角度回绕）。

## 2. 做法：`_attitude_govern(v)`

在 `px4_iface` 里，`publish_velocity` 于正常态（`OK`）末尾调用：

```
tilt  = max(|roll|, |pitch|)
s_tilt = scale01(tilt, tilt_soft, tilt_hard)     # soft→1, hard→0, 中间线性
s_rate = scale01(|ω|, rate_soft, rate_hard)      # ω 由 attitude 差分
s = min(s_tilt, s_rate)
水平指令 *= s                                     # 倾角/角速率越大，水平需求越小
水平指令变化率 ≤ accel_h_max/hz · max(s_tilt, 0.15)   # 指令加速度限额，姿态越差额度越小
```

- 到硬限（默认 40°）时**水平指令归零**，让姿态控制器回正；垂直指令不改。
- **只在 `_safety_state=='OK'` 生效**，不回退 HOLD/PULLBACK 等安全指令。
- 记录 `_att_gov`（缩放因子）供日志/观测。

## 3. 新增参数（launch 可覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `attitude_constraint_enable` | `true` | 总开关 |
| `tilt_soft_deg` | `25.0` | 倾角软限：超过则衰减水平指令 |
| `tilt_hard_deg` | `40.0` | 倾角硬限：达到则水平指令=0 |
| `rate_soft_dps` | `150.0` | 角速率软限 (deg/s) |
| `rate_hard_dps` | `300.0` | 角速率硬限 (deg/s) |
| `accel_h_max` | `5.0` | 水平指令加速度上限 (m/s²) |

`b_node` 周期日志新增 `att=<缩放>`。

## 4. 验证（2026-10-05）

单元（直接调 `_attitude_govern`，`accel_h_max=0` 隔离）：
```
tilt 10°   -> [2,0,1]  s=1.0   （无衰减）
tilt 32.5° -> [1,0,1]  s=0.5   （水平减半）
tilt 45°   -> [0,0,1]  s=0.0   （水平归零）
rate 225°/s-> [1,0,1]  s=0.5
加速度限额: 从 [0,0,1] 发 [3,0,1] -> [0.1,0,1]（水平≤5/50=0.1）
```
SITL（标称 M6）：
```
B phase=... safe=OK att=1.00   （27 个采样全为 1.00，全程未误触发）
*** STACK CAPTURED horiz=0.032m ，双机落地、无 failsafe
SAFETY 仅启动期 estimator_reset → 1s HOLD 自愈
```

## 5. 局限（诚实说明）

- 这是**缓解**：通过限制姿态需求，让 EKF/磁罗盘尽量不进入发散区；**根因（本机 mag/EKF 鲁棒性）
  仍需平台侧修复**（C1），否则 `v_p≈4.9 m/s` 的硬边界依然存在。
- 角速率来自低速率 attitude 差分，噪声较大，故只用于**软约束**（阈值故意设高，标称不触发）。

改动文件：`payload_catch/px4_iface.py`、`b_node.py`、`launch/catch_stack_launch.py`。
