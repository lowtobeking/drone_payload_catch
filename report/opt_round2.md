# 优化 第二轮：安全 / 控制 / 协调（分级、低风险）

> 背景：`report/optimization_backlog.md` 指出三块（协调/安全/控制）里仍有空白。
> 本轮按"性价比"顺序实施，**每项都保持默认行为不破坏既有结论**，新增能力大多为 opt-in 或只在异常时触发。
>
> 复现：离线 `python3 tools/stack_run.py ...`；SITL `bash run_m6_sitl.sh 70`（见各节）。
>
> 进度：**#1 ✅ · #2 ✅ · #3 ✅ · #4 ✅ · #5 ✅ · #6 ✅**

---

## #1 安全状态机：HOLD / PULLBACK / LAND / KILL + 越界回拉 ✅

### 1.1 问题
原 `px4_iface._safety_check` 是**二值**响应：检测异常（姿态超限/越界/状态超时）→ 要么只报错、
要么直接 `MAV_CMD_DO_FLIGHTTERMINATION`（切动力下坠）。缺中间档：
`report/safety_control_review.md` 的 **G1（越界回拉）/ G2（丢目标回退悬停）** 是空白；
`min_relA` 只监测不动作。

### 1.2 做法
把安全监督升级为**分级状态机**（`payload_catch/px4_iface.py`，A/B 共用基类）：

```
OK ──越界──▶ PULLBACK（软回拉，可恢复）
   ──状态超时──▶ HOLD（原地悬停，可恢复）
   ──姿态临界(持续 kill_hold_s)──▶ LAND  或  KILL（取决于 safety_auto_kill / escalate）
持续 HOLD ──(opt-in)──▶ LAND ──▶ KILL
```

- **PULLBACK**：`_pullback_velocity()` 把世界系位置软拉回围栏内
  （xy 回到 `±safety_geofence_xy`，高度回到 `[safety_alt_min, safety_geofence_alt]`），
  速度限 `safety_pullback_speed`。替代"越界就 kill"。
- **HOLD**：`publish_velocity()` 把任务速度覆盖为 **0**（就地刹停悬停），**任务逻辑仍运行**
  （A 继续广播状态/意图，B 继续估计），异常消除后自动回 `OK`。
- **LAND**：`request_land()` 交接 PX4 `AUTO_LAND`；可外部/子类调用，或由 `safety_hold_escalate='land'` 自动升级。
- **KILL**：仍为最后手段（`safety_auto_kill=true` 时临界异常持续即终止），且 `kill()` 现在把状态标为 `KILL`。

### 1.3 新增参数（launch 可覆盖，默认保守不改变正常行为）

| 参数 | 默认 | 说明 |
|---|---|---|
| `safety_pullback_enable` | `true` | 越界时主动回拉 |
| `safety_pullback_k` | `1.0` | 回拉比例增益 |
| `safety_pullback_speed` | `2.0` | 回拉速度上限 m/s |
| `safety_alt_min` | `-1.0` | 高度下界 (m) |
| `safety_hold_escalate` | `none` | 持续 HOLD/临界后升级：`none`\|`land` |
| `safety_hold_timeout` | `8.0` | HOLD 多久后升级 (s) |
| `safety_auto_kill` | `false` | 临界异常持续即飞行终止（原参数） |

`launch/catch_stack_launch.py` 已把上述透传给 A/B。M1 `catch_launch.py` 用默认值即可。

### 1.4 验证
ROS 冒烟测试（无需 PX4，直接驱动状态迁移）：

```
1 normal              state=OK
2 geofence xy         state=PULLBACK   pullback vel=[-1.715,-1.029,0]（范数≈2，方向正确）
3 geofence alt        state=PULLBACK
4 recover             state=OK
5 pos stale           state=HOLD
6 escalate(land)      state=LAND       _landing=True
7 critical+autokill   killed=True      state=KILL
8 HOLD                state=HOLD（publish_velocity 覆盖为 0）
```

离线层与 `px4_iface` 导入均正常：
`python3 -m payload_catch.payload_model` / `rendezvous` / `tools/stack_run.py` 不变。

### 1.5 SITL 验证（已跑）

| 测试 | 命令 | 结果 |
|---|---|---|
| 标称任务 | `bash run_m6_sitl.sh 70` | `STACK CAPTURED horiz=0.008m rel_v=2.330`、双机落地、**无 SAFETY、无 failsafe** |
| 越界回拉（空中） | `LAUNCH_EXTRA="safety_geofence_alt:=3.0"` | `SAFETY PULLBACK`（A/B 均被压到 3.0m）；滞环后仅 4 次 PULLBACK/3 次恢复（不再抖动） |
| 丢失状态看门狗 | 同上（启动瞬态） | `SAFETY HOLD: pos_stale=2.8s` → `恢复 → OK`（HOLD 生效并自动恢复） |
| 外部 kill | （既有验证） | 单机飞行终止，另一机不受影响 |

注：回拉会临时接管升力（`want_alt` 压到围栏），故在**地面且已超界**的极端配置下会把机原地压住——属预期（不飞得更远）。

---

## #2 控制：加速度前馈（用上 `_ar`）+ 速度设定点速率限幅 ✅

### 2.1 问题
`b_node` DIVE 里 `_stack_ref` 返回的**参考加速度 `_ar` 被丢弃**（`pr, vr, _ar = ...`），SITL 只用
`v_sp = vr + kp·(pr−pos)`；而离线 `simulate_stack` 用的是 `a_cmd = ar + kp·(pr−p) + kd·(vr−v)`。
两者不一致，且 SITL 缺了前馈 → 下潜/刹车段存在跟踪滞后。

### 2.2 关键发现（PX4 源码）
velocity 模式下 PX4 会把 `trajectory_setpoint.acceleration` **作为前馈直接叠加**：
```
// src/modules/mc_pos_control/PositionControl/PositionControl.cpp::_velocityControl
acc_sp_velocity = vel_error * gain_vel_p + vel_int - vel_dot * gain_vel_d;
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);   // _acc_sp 来自 acceleration 字段
```
且 `control_mode.cpp` 的 velocity 分支会置 `flag_control_acceleration_enabled = true`。
所以**无需改 offboard 模式**，只要把 `acceleration` 从 NaN 换成 `ar` 即可。

### 2.3 做法
- `px4_iface.publish_velocity(vel, yaw, acc_ff=None)` 新增 `acc_ff` 参数：非 None 时写入
  `TrajectorySetpoint.acceleration`（否则保持 NaN，行为不变）。
- `b_node` DIVE：`pr, vr, ar = _stack_ref(...)`，`acc_ff = a_ff_gain * ar` 下发。
- 新增通用 `sp_rate_limit`（速度设定点变化率上限 m/s²，默认 0=不限），在 `publish_velocity`
  里对指令做变化率限幅（平滑、降姿态激励）。

### 2.4 新增参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `a_ff_gain` | `1.0` | DIVE 加速度前馈增益（0=关） |
| `sp_rate_limit` | `0.0` | 速度设定点变化率上限 (m/s²)，0=不限 |

### 2.5 验证
- 消息捕获（ROS）：DIVE 前馈字段正确写入 `acceleration`。
- SITL：`a_ff_gain=1.0` 标称任务 `STACK CAPTURED horiz=0.008m`（优于历史 0.025–0.13m）、无 failsafe。
- 离线层不变。

---

## #3 感知协同：B 在线 σ 上传给 A 的释放余量闸 ✅

### 3.1 问题
A 的释放余量闸用**静态** `release_sigma=0.03`；B 其实实时算了在线相对定位不确定度
`_sigma_est`（测量与 EMA 估计的残差），但只用于 keep-out，**没传给 A**。

### 3.2 做法
- B `_pub_ready` 的消息扩为 `[ready, rel_xy, spd_xy, stamp, sigma]`（第 5 位）。
- A `_on_b_ready` 解析 sigma（兼容旧 4 元格式）；余量闸用
  `sigma = clamp(max(release_sigma, b_sigma), ., release_sigma_max)`。
- 新增 `use_b_sigma`（默认 true）、`release_sigma_max`（默认 0.15，防重噪声把闸门卡死）。
- 放上释放日志：`release@... (lead=..., σ=...)`。

### 3.3 验证
- 解析/兼容单测：5 元→`_b_sigma=0.07`；4 元旧格式不崩且保留旧值。
- SITL（handshake）：`release@... σ=0.030`（B 在线值），`STACK CAPTURED horiz=0.080m`，无 failsafe。
- 重噪声下闸门更保守→A 保留载荷落地（安全侧），而非投出去接不住。

---

## #4 协调：释放提交窗口 + lead 窗口取消（补 G3）✅

### 4.1 问题
原 handshake 在**单拍** gate 通过时就提交释放（`now+release_lead`），单拍抖动可能误投；且一旦提交，
`release_lead` 到真正释放的窗口内无法收回。

### 4.2 做法
- **提交窗口**：gate 通过后不立即释放，需**持续** `commit_hold_s`（默认 0.20s）才提交。
- **lead 窗口复核/取消**：提交后到 `t_rel` 前每拍复核；若 A 自身不再就位 →
  `_abort_release()`：发 `/payload/release_abort` + `release_cmd=[-1,..]`，重置状态可重试。
- `payload_node` 新增 `/payload/release_abort`：撤掉已排定的释放时刻（未释放时）。
- `b_node` 收到 `release_cmd<0` 且尚未下潜（`not _dive_anchored`）→ 回 `ALIGN`。
- 关键修正：B ack 后进 DIVE 会**停发 ready**，故 lead 窗口只复核 **A 自身就位状态**（不看 ready），
  否则会把正常释放误取消。

### 4.3 新增参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `commit_hold_s` | `0.20` | 就绪门限需持续多久才提交释放 |

### 4.4 验证（ROS 单测，无需 PX4）
```
1) 提交窗口启动: _commit_t0=1000.0 released=False
2) 持续 0.25s 后提交: released=True _released_at=1000.5
3) lead 窗口内 A 未就位 → 取消: released=False _released_at=None
4) 大 σ 余量不足 → 不进入提交窗口
5) 0.1s<hold 不提交；0.25s>hold 提交
```
SITL（handshake）：`就绪门限通过 → 提交窗口` → `释放权威发布 release@... σ=0.030` → `STACK CAPTURED horiz=0.031m`。

---

## #5 安全：3D 反应式 keep-out（`min_relA` 从“监测”变“有动作”）✅

### 5.1 问题
静态 keep-out 只约束“B 高度 ≤ A−gap_eff”；`min_relA` 每拍算但只打印。水平贴近/异常机动时
没有真实三维距离的防护。

### 5.2 做法
在 `b_node` 覆写 `publish_velocity`，栈模式下统一叠加 `_keepout_velocity`：
- `dist = |p_A − p_B|`；若 `dist < keepout_dist`，去掉速度指令中**指向 A 的分量**，
  再叠加 `keepout_gain·(keepout_dist − dist)` 的**排斥**。
- 与高度规则互补（后者管垂直，前者管三维），且只在接近时生效，不影响标称（A/B 距~0.9–1.5m）。

### 5.3 新增参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `keepout_enable` | `true` | 启用 3D 反应式 keep-out |
| `keepout_dist` | `0.60` | 触发距离 (m) |
| `keepout_gain` | `1.0` | 排斥增益 (1/s) |

### 5.4 验证（ROS 单测）
```
v_cmd=[2,0,0]（朝 A），dist=0.3 < 0.6  → keepout=[-0.3,0,0]（去内向 + 排斥）
dist=2.0 (>0.6) → 不变；a_est=None → 不变
```

---

## #6 控制：在线自适应下潜 ✅

### 6.1 问题
`stack_drop._adaptive_dive`（滚动重解 a_dive，抗下击暴流）只跑在离线 `--vert adaptive`；
`b_node` 只在进入 DIVE 时算一次 `plan_stack_drop`，在线不重解。

### 6.2 做法
- `b_node` 新增 `adaptive_dive`（默认 false）：DIVE 且已重锚后，每拍用估计的载荷竖直状态
  调 `_adaptive_dive(...)` 重解目标 a_dive，**限速 slew=20/s** 平滑，然后以短视界
  `look=4/hz` 覆写垂直参考 `pr/vr/ar`（与离线 `--vert adaptive` 同构）。
- 载荷竖直状态优先用 `_payload_est`（含测量噪声/延迟），否则用 `self.p_pay`。

### 6.3 新增参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `adaptive_dive` | `false` | 在线自适应下潜 |
| `adaptive_alt_floor` | `0.35` | 刹停后最小离地 (m) |

### 6.4 验证
- 离线 `python3 tools/stack_run.py --vert adaptive` 结果不变（函数未改）。
- SITL `adaptive_dive:=true`（叠加 handshake）：`STACK CAPTURED horiz=0.035m`、双机 `Landing detected`、无 failsafe。

---

## 总结与集成验证

一次集成 SITL（handshake + adaptive_dive）同时覆盖 #3/#4/#5/#6：
```
A: 就绪门限通过 → 进入释放提交窗口 (0.20s)
A: 释放权威发布 release@... (lead=0.2, σ=0.030)
B: 收到 A 释放 ack → DIVE → 重锚 → *** STACK CAPTURED horiz=0.035m ***
A/B: LAND → Landing detected（无 failsafe）
（启动瞬态 SAFETY HOLD: pos_stale=3.0s → 自动恢复 OK）
```

## 新增/变更参数一览

| 模块 | 参数 | 默认 |
|---|---|---|
| 安全 | `safety_pullback_enable` / `_k` / `_speed` / `_clear` / `_alt_min` / `_geofence_xy` / `_geofence_alt` / `_hold_escalate` / `_hold_timeout` | true / 1.0 / 2.0 / 0.15 / -1.0 / 50 / 30 / none / 8.0 |
| 控制 | `a_ff_gain` / `sp_rate_limit` | 1.0 / 0.0 |
| 协同 | `use_b_sigma` / `release_sigma_max` / `commit_hold_s` | true / 0.15 / 0.20 |
| 安全 | `keepout_enable` / `keepout_dist` / `keepout_gain` | true / 0.60 / 1.0 |
| 控制 | `adaptive_dive` / `adaptive_alt_floor` | false / 0.35 |
