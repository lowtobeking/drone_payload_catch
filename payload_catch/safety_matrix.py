#!/usr/bin/env python3
"""safety_matrix.py —— 安全层**故障注入矩阵**的场景规格与判定（纯逻辑，无 ROS）。

对标 drone_package_20260908 的 `safety_filter_SITL回归清单.md`（S27–S33）：
既要确认安全层**没帮倒忙**（正常飞行零误触发），也要确认**每道保护真能触发**。

- `SCENARIOS`：每个场景的注入方式（launch 参数/中途动作）与期望日志（正则 present/absent）。
- `verdict(log_text, scenario)`：纯函数判定 PASS/FAIL。
- 编排器 `tools/sitl_safety_matrix.py`；单测 `tools/test_safety_matrix.py`。
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field


@dataclass
class Scenario:
    name: str
    sec: int = 50                     # SITL 运行秒数
    launch_extra: str = ''            # LAUNCH_EXTRA
    action: str = ''                  # '' | 'kill_peer' | 'kill_b'（中途动作）
    action_after_ready_s: float = 14.0
    expect_present: list = field(default_factory=list)   # 必须匹配的正则
    expect_absent: list = field(default_factory=list)    # 必须不匹配的正则
    why: str = ''


SCENARIOS = [
    Scenario(
        name='baseline_no_false_trigger', sec=50,
        expect_present=[r'STACK CAPTURED'],
        expect_absent=[r'collision_floor', r'SAFETY PULLBACK', r'失联',
                       r'SAFETY KILL', r'Failsafe activated'],
        why='正常 M6：安全层零误触发、任务成功'),

    Scenario(
        name='geofence_pullback', sec=45, launch_extra='safety_geofence_alt:=3.0',
        expect_present=[r'SAFETY PULLBACK'],
        expect_absent=[r'Failsafe activated'],
        why='高度围栏压到 3m → 越界回拉（可恢复），不触发平台 failsafe'),

    Scenario(
        name='collision_floor_hold', sec=45,
        launch_extra='collide_warn:=1.7 collide_emerg:=1.5',
        expect_present=[r'SAFETY HOLD: collision_floor'],
        why='把碰撞地板抬到正常间距之上 → 硬地板触发并升 HOLD'),

    Scenario(
        name='peer_loss_freeze_land', sec=55,
        launch_extra='peer_loss_hold_s:=1.0 peer_loss_land_s:=4.0',
        action='kill_peer', action_after_ready_s=14.0,
        expect_present=[r'失联 .*就地冻结', r'安全悬停并降落'],
        why='飞行中丢 A 状态 → 先冻结（不追陈旧参考）→ AUTO.LAND'),

    Scenario(
        name='kill_external', sec=45, action='kill_b', action_after_ready_s=12.0,
        expect_present=[r'SAFETY KILL'],
        why='外部 /safety/kill_b → B 飞行终止（A 不受影响）'),
]


def verdict(log_text: str, sc: Scenario):
    """纯函数：按场景的 present/absent 正则判定。返回 (ok, detail, matched)。"""
    missing = [p for p in sc.expect_present if not re.search(p, log_text)]
    forbidden = [p for p in sc.expect_absent if re.search(p, log_text)]
    matched = [p for p in sc.expect_present if re.search(p, log_text)]
    ok = (not missing) and (not forbidden)
    parts = []
    if missing:
        parts.append('缺: ' + ','.join(missing))
    if forbidden:
        parts.append('误触发: ' + ','.join(forbidden))
    detail = '；'.join(parts) if parts else ('命中: ' + ','.join(matched) if matched else '')
    return ok, detail, matched


def _selftest() -> bool:
    ok = True

    def chk(name, cond):
        nonlocal ok
        print(f'  {"✅" if cond else "❌"} {name}')
        ok = ok and bool(cond)

    sc = Scenario('t', expect_present=[r'STACK CAPTURED'],
                  expect_absent=[r'collision_floor'])
    chk('命中 present + 无 absent → PASS', verdict('... STACK CAPTURED ...', sc)[0])
    chk('缺 present → FAIL', not verdict('nothing here', sc)[0])
    chk('出现 absent → FAIL', not verdict('STACK CAPTURED\ncollision_floor', sc)[0])
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
