# 安全监督：飞行异常时可 kill（飞行终止）

> 需求：**飞行异常时能够 kill**（切动力/终止飞行）。已在共享基类 `px4_iface.Px4Drone` 实现，
> A/B 都具备；支持**外部 kill 话题**与**异常自动 kill**，并已在 SITL 验证。

---

## 1. 机制

| 能力 | 实现 |
|---|---|
| **外部 kill** | 每机订阅 `/safety/kill_a`（A）、`/safety/kill_b`（B）；收到 `Bool(true)` → 立即飞行终止 |
| **飞行终止命令** | `kill()` 下发 PX4 `MAV_CMD_DO_FLIGHTTERMINATION(=185, param1=1)` 并停发 offboard setpoint |
| **异常检测（自动）** | `_safety_check()` 每拍检查：①姿态角超限 ②水平/高度越界 ③位置状态超时；异常**持续** `safety_kill_hold_s` 后 →（若 `safety_auto_kill`）自动 kill，否则仅报错 |
| **参数** | `safety_lock`、`safety_kill_topic`、`safety_auto_kill`、`safety_tilt_max_deg`、`safety_geofence_xy/alt`、`safety_state_timeout`、`safety_kill_hold_s` |
| **健全性** | 每机独立话题（不会误杀另一机）；仅 `armed` 时判定；异常需**持续**才触发；kill 只发一次；打印 reason |

---

## 2. SITL 验证（外部 kill）

流程：启动 M6 SITL → 双机起飞 → 向 `/safety/kill_b` 发一次 `Bool(true)`。

```
$ ros2 topic pub --once /safety/kill_b std_msgs/msg/Bool "{data: true}"
[b_node-2] [ERROR] [1] *** SAFETY KILL（飞行终止）*** reason=external /safety/kill
px4_1(B): Preflight Fail: Flight termination active        ← B 动力被切
px4_0(A): Flight termination 计数 = 0                       ← A 未被误杀（仍在飞）
```

**结论：每机 kill 正确**——B 被终止，A 不受影响。

---

## 3. 与《保护控制审查》缺口的对照

本次补上：
- **G2 丢失目标/状态**：`safety_state_timeout`（位置状态超时 → 异常；可自动 kill）。
- **G1 Geofence**：`safety_geofence_xy/alt` 越界检测（**检测+可 kill**，尚未做"回拉"）。
- **兜底 kill**：异常时的手动/自动飞行终止。

仍缺（见 `report/safety_control_review.md`）：
- **G1 越界回拉**（目前只检测/kill，不主动拉回）。
- **G3 接不住的 abort/go-around**、**G4 反应式避碰**、**G5 低电/超时任务中止**、**G7 捕获失败处置**。

---

## 4. 用法

```bash
# 手动 kill（单机）
ros2 topic pub --once /safety/kill_b std_msgs/msg/Bool "{data: true}"

# 自动 kill（异常持续即终止）：launch 参数
#   safety_auto_kill:=true  safety_tilt_max_deg:=45  safety_kill_hold_s:=0.5
```

> ⚠️ 飞行终止会让载机**失去动力下坠**，是**最后手段**；默认 `safety_auto_kill=false`（只报错），
> 需要时再手动/显式开启。
