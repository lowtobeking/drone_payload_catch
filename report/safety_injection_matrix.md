# 安全层故障注入 SITL 矩阵

> 编排：`tools/sitl_safety_matrix.py`；判定：`payload_catch/safety_matrix.py`（纯逻辑）。
> 对标参考工程 `safety_filter_SITL回归清单.md`（S27–S33）。

| 场景 | 注入 | 期望 | 结果 | 证据/原因 |
|---|---|---|---|---|
| `baseline_no_false_trigger` | 标称 | 正常 M6：安全层零误触发、任务成功 | ✅ PASS | 命中: STACK CAPTURED |
| `geofence_pullback` | safety_geofence_alt:=3.0 | 高度围栏压到 3m → 越界回拉（可恢复），不触发平台 failsafe | ✅ PASS | 命中: SAFETY PULLBACK |
| `collision_floor_hold` | collide_warn:=1.7 collide_emerg:=1.5 | 把碰撞地板抬到正常间距之上 → 硬地板触发并升 HOLD | ✅ PASS | 命中: SAFETY HOLD: collision_floor |
| `peer_loss_freeze_land` | peer_loss_hold_s:=1.0 peer_loss_land_s:=4.0 | 飞行中丢 A 状态 → 先冻结（不追陈旧参考）→ AUTO.LAND | ✅ PASS | 命中: 失联 .*就地冻结,安全悬停并降落 |
| `kill_external` | kill_b | 外部 /safety/kill_b → B 飞行终止（A 不受影响） | ✅ PASS | 命中: SAFETY KILL |

**汇总：5/5 PASS**