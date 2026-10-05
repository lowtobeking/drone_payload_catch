# 优化第三轮：安全硬化（低成本）

> 承接 `report/opt_round2.md`。本轮只做**低成本、低风险**的安全硬化，直接加固第二轮成果。
> 进度：**A1 ✅ · A4 ✅ · A6 ✅**

---

## A1 安全指令绕过速率限幅 ✅

### 问题
`publish_velocity` 里先做安全覆盖（HOLD/PULLBACK）再对结果做 `sp_rate_limit`。将来若开启限幅，
**安全刹车/回拉会被平滑拖慢**。

### 做法
速率限幅只在 **`_safety_state == 'OK'`** 时生效；安全指令（HOLD/PULLBACK）不经过限幅。
同时每次都更新 `_last_vsp`，避免恢复 OK 时跳变。

### 验证
单元：`sp_rate_limit=0.5`、上一指令 `[5,0,0]`、状态 `HOLD` → 立即发出 `[0,0,0]`（未被限幅）。

---

## A4 围栏改为"软限幅"（消除与任务互相顶）✅

### 问题
原 PULLBACK 直接**替换**为纯回拉速度，会连切向机动一起刹掉，且安全层持续与任务"顶"在边界。

### 做法
新增 `px4_iface._fence_velocity(task_v)`：在**任务速度**上
- 越界轴：强制向内（`k·越界量`）；
- 界内轴：只**剔除继续向外**的分量，保留切向；
- 最后按 `safety_pullback_speed` 限幅。

PULLBACK 现在走软限幅（`HOLD` 仍是原地悬停）。既保证不飞出，又不损失沿围栏的切向机动。

### 验证（单元）
```
界内 p>0: [1,2,0]  -> [0,2,0]   （去掉向外 x，保留切向 y）
越界 x:   [1,0,0]  -> [-2,0,0]  （强制向内）
alt 过高: [0,0,-1] -> [0,0,2]   （禁止继续爬、强制下压）
alt 过低: [0,0,1]  -> [0,0,-1]  （强制上拉）
```
SITL（`safety_geofence_alt:=3.0`）：A/B 被稳定压在 3.0m，`safe=PULLBACK` 不再抖动；
标称（fence 50）无影响：`STACK CAPTURED horiz=0.018m`。

---

## A6 安全状态进周期日志 ✅

- `b_node` 周期日志增 `safe={self._safety_state}`。
- `a_node` 新增 1s 周期日志：`A safe=... pos_w=... vel=... released=...`。
- 便于回归/调试时一眼看到是否处于 HOLD/PULLBACK。

---

## 测试记录（2026-10-05）

| 测试 | 结果 |
|---|---|
| 软围栏/限幅旁路单元 | 全通过（见上） |
| SITL 高度围栏 3.0m | `SAFETY PULLBACK` 稳定、`safe=` 日志可见 |
| SITL 标称 | `STACK CAPTURED horiz=0.018m`，无 SAFETY 事件、无 failsafe |

改动文件：`payload_catch/px4_iface.py`、`payload_catch/a_node.py`、`payload_catch/b_node.py`。
