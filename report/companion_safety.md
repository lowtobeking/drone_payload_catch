# companion 安全网：失联看门狗 + 硬碰撞地板（对标参考工程 safety_filter）

> 对标 drone_package_20260908 的 `mpc_control/safety_filter.py`（companion 侧独立安全层）与
> “失联自动降落”第 2 层。本轮补上我们最缺的两块，均已 **纯逻辑单测 + SITL 诱发验证**。

---

## 1. 失联看门狗：丢 A 状态 → 就地冻结 → AUTO.LAND

参考工程第 2 层：companion 健康、仍在发 setpoint，只是收不到 leader → 飞控永不失联 →
必须在 companion 侧补看门狗。我们的 B 依赖 A 的 `/drone_a/state`（相对定位），若丢失：

```
[纯逻辑] payload_catch/safety_logic.peer_loss_action(since_last_rx, hold_s, land_s)
  → 'none' | 'hold' | 'land'      （hold_s<=0 关闭；约束 0<hold_s<land_s）
[b_node] control() 每次：若 now-最近收到 A>hold → 就地悬停（不追陈旧参考）并 return；
         若 >land → request_land('comms loss')。恢复则退出降级。
```

参数（默认 **0=关**，不影响既有行为）：`peer_loss_hold_s`、`peer_loss_land_s`。

**SITL 实证**（`LAUNCH_EXTRA="peer_loss_hold_s:=1.0 peer_loss_land_s:=4.0"`，飞行中 kill A 节点）：
```
B: 失联 1.0s 未收到 A 状态 → 就地冻结（不追陈旧参考）
B: 失联 4.0s → 安全悬停并降落
[1] SAFETY LAND: comms loss (A state stale)
```

## 2. 硬碰撞地板：太近 → 去掉朝 A 的速度分量 + 持续 → HOLD

参考工程缺口 2「硬碰撞地板」：用实测邻居距离独立硬判，太近横向刹停（绕过软代价）。

```
[纯逻辑] payload_catch/keepout.hard_floor(v_sp, r, d_warn, d_emerg)
  r = A−B（指向 A）；d<d_warn 去掉朝 A 分量；d≤d_emerg 记为 emerg
[b_node] publish_velocity（stack 模式）叠加：keepout/CBF 之后再过 hard_floor；
         连续 emerg ≥ collide_hold_frames → 注入 external_safety_reasons → 安全状态机 HOLD
[px4_iface] external_safety_reasons 计入安全状态机（可恢复 HOLD，不让 PULLBACK 抢）
```

参数：`collide_warn`(0.90m)、`collide_emerg`(0.50m)、`collide_hold_frames`(15)。
默认值低于 M6 正常 A-B 3D 间距（≈ gap ≥ 1.0m），**标称不触发**。

**SITL 实证**：
- 标称（默认阈值）：`STACK CAPTURED horiz=0.015m`，无 `collision_floor`（零误触发）。
- 诱发（`LAUNCH_EXTRA="collide_warn:=1.7 collide_emerg:=1.5"`）：
  ```
  SAFETY HOLD: collision_floor(d=1.43)   → 全程 safe=HOLD，B 速度≈0（不再接近）
  ```

## 3. 与我们已有保护的关系

- 既有：`min_ab_gap` 静态高度 keep-out、`keepout_dist` 3D 排斥、CBF(opt-in)。
- 新增硬地板是**最后一闸**：软约束失效（估计跳变/异常机动）时仍不接近；并与安全状态机联动（HOLD）。
- 失联看门狗是**任务级失效恢复**：补上"收不到 A 就傻等/追陈旧参考"的洞。

## 4. 复现

```bash
source env.sh
# 失联看门狗（飞行中 kill A 节点）
LAUNCH_EXTRA="peer_loss_hold_s:=1.0 peer_loss_land_s:=4.0" bash run_m6_sitl.sh 55
# 诱发碰撞地板
LAUNCH_EXTRA="collide_warn:=1.7 collide_emerg:=1.5" bash run_m6_sitl.sh 45
# 纯逻辑
python3 tools/test_keepout.py && python3 tools/test_safety_logic.py
```
