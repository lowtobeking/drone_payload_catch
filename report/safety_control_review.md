# 保护/安全控制审查（现有 vs 缺口）

> **更新（2026-10-09）**：本页是 2026-10-04 的初始审查。此后多数缺口已补：
> G1/G2/G4/G6/G8 → `report/opt_round2..5.md`（分级状态机/回拉/看门狗/3D keep-out/姿态滤波）；
> **G5 低电** + **飞控参数体检/失效保护/围栏预置** → `report/fcu_safety.md`；
> 接空中止 → `report/dynamics_contact.md`。仍缺：真机/HIL、接空失败物理防护、跨机冗余。
>
> 问题："有做任何的保护控制吗？" 结论：**有，但只是局部的命令限幅 + 静态/不确定度 keep-out + 释放前闸**；
> 缺少**边界保护（geofence）、丢失目标看门狗、接不住的 abort/go-around、反应式避碰、低电/超时、捕获失败处置**。

---

## 1. 已有的保护控制（代码级）

| # | 保护 | 位置 | 说明 |
|---|---|---|---|
| 1 | **指令限幅（速度/爬升）** | `px4_iface.hover_velocity` | XY 速度范数 ≤ `max_speed`；Z ∈ [−max_climb, +max_climb] |
| 2 | **DIVE 指令限幅** | `b_node` DIVE | `v_sp[:2]` 范数 ≤ `v_max`；`v_sp[2]` clip 到 ±`v_max` |
| 3 | **规划可行性约束** | `stack_drop.plan_stack_drop` | `v_rel ≤ v_retain`、`a_dive < g`、接触高度 ≥ `ground_margin`、刹车后离地 ≥ `ground_margin` |
| 4 | **静态 keep-out** | `b_node` 所有 phase | B 高度 ≤ A 高度 − `min_ab_gap`（默认 0.8m），**绝不靠近 A** |
| 5 | **不确定度 keep-out** | `b_node._gap_eff` | `min_ab_gap + kσ`（在线估 σ）；WAIT_A 会主动下降满足 |
| 6 | **释放前闸** | `a_node._coord_tick` | 余量闸（`rel_xy+漂移+kσ ≤ eff_r−margin`）+ 一致性门限 + 就位判定 |
| 7 | **握手时序保护** | `a_node`/`b_node` | ready 新鲜度 `ready_timeout`、ack 超时 `handshake_timeout`、单次释放、ack 原子事件 |
| 8 | **释放后清场** | `a_node` | 释放后 A 移到 `hover+clear_offset`，给 B 让空域 |
| 9 | **碰撞监测** | `b_node` | 全程打印 `min_relA`（**仅监测，无动作**） |
| 10 | **平台层** | `px4_iface` | offboard 心跳；`land()` 发 `AUTO_LAND`；PX4 自身 failsafe（外部） |
| 11 | **"只下潜不爬升"** | `stack_drop.simulate_stack`（离线显式） | 离线禁止 B 主动爬升；SITL 由 z 参考隐含 |
| 12 | **安全监督 + 飞行终止(kill)** | `px4_iface.Px4Drone`（A/B 共享） | 外部 kill 话题 `/safety/kill_a|b`；异常（姿态超限/越界/状态超时）持续即（可）`MAV_CMD_DO_FLIGHTTERMINATION`。见 `report/safety_supervisor.md` |

---

## 2. 缺失的保护控制（缺口）——**状态更新（2026-10-09）**

| # | 缺口 | 现状 | 证据/位置 |
|---|---|---|---|
| G1 | **Geofence（位置/高度边界）** | ✅ 软围栏 + **越界回拉**（可恢复） | `px4_iface._fence_velocity`；`opt_round2/3`；矩阵 `geofence_pullback` PASS |
| G2 | **丢失目标/A 的看门狗** | ✅ **失联看门狗**：丢 A→就地冻结→AUTO.LAND | `safety_logic.peer_loss_action`+`b_node`；`companion_safety.md`；矩阵 PASS |
| G3 | **接不住的 abort / go-around** | ✅ 释放前余量闸 + 编队超时 abort + 接空 `MISS`→悬停→降落 | `a_node._abort_release`/`b_node._abort_formation`/`miss_timeout_s` |
| G4 | **反应式避碰（非静态规则）** | ✅ 3D keep-out + CBF(opt-in) + **硬碰撞地板**→HOLD | `b_node._keepout_velocity`/`_cbf_velocity`/`keepout.hard_floor`；矩阵 PASS |
| G5 | **低电/超时保护** | ✅ **电池 warning≥2/remaining<0.07→LAND** + 起飞前电池门 | `safety_logic.battery_*`+`px4_iface`；`fcu_safety.md` |
| G6 | **姿态/倾角/jerk 限制** | 🟡 倾角/角速率/指令加速度 ✅；**显式 jerk 未做** | `px4_iface._attitude_govern`+`sp_rate_limit`；`opt_round5` |
| G7 | **捕获失败后处置** | 🟡 接空 `MISS`→悬停→降落 ✅；**载荷砸地物理防护/告警未做** | `b_node` MISS；物理防护待真机 |
| G8 | **B 的加速度/推力约束显式化** | ✅ 四旋翼聚合约束 + 指令加速度限额 | `payload_catch/dynamics.py`；`dynamics_contact.md` |
| G9 | **通信/时钟异常保护** | 🟡 时钟 ping/pong + `peer_loss` ✅；消息乱序未专门处理 | `a_node/b_node` clock_sync；`companion_safety.md` |

> 总览与与参考工程对照：`report/safety_overview.md`。

---

## 3. 建议（按性价比）

1. **G2 丢失目标看门狗**（最简单、最该有）：给 A 状态/载荷估计加超时；超时 → 原地悬停（或降落）。
2. **G1 简化 geofence**：对 A/B 的 xy/高度做软限幅与越界处理。
3. **G3 可捕获性判定**：释放前用落点余量闸（已有雏形）+ 预测 B 能否在 τ 内到位；否则不释放。
4. **G6 倾角/加加速度限幅**：既保护姿态（已知 failsafe 边界），又提升平滑性。
5. **G4 反应式避碰**：把 `min_relA` 从"监测"升级为"动作"（接近阈值时排斥）。

---

## 4. 一句话

现有保护集中在**"指令限幅 + 空间 keep-out + 释放前闸 + 时序"**；
**"边界、失效恢复、动态避碰、任务级中止"仍是空白**——这些是下一阶段"保护控制"的主要工作。
