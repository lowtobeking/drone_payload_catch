#!/usr/bin/env python3
"""safety_logic.py —— 安全层**纯函数**（无 ROS 依赖，可离线单测）。

从 `px4_iface.Px4Drone` 抽出，目的与参考工程 `frame_convert.py` 相同：
把"安全状态机 / 围栏 / 姿态滤波 / 传感器健康"这些**容易翻车、却又在仿真里
看起来能飞**的逻辑，做成可脱离 ROS/PX4/Gazebo 直接跑的纯函数，改代码后必跑
（见 `tools/test_safety_logic.py` 与 `tools/run_checks.sh`）。

约定：世界系 NED（x=北, y=东, z=下），高度 = `-z`。
"""
from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence

import numpy as np


# --------------------------------------------------------------- 基本工具
def scale01(x: float, soft: float, hard: float) -> float:
    """软/硬限之间的线性缩放：x≤soft→1，x≥hard→0。"""
    if hard <= soft:
        return 1.0 if x <= soft else 0.0
    return float(np.clip((hard - x) / (hard - soft), 0.0, 1.0))


def clip_to_estimator_limits(v, vxy_max: float, vz_max: float) -> np.ndarray:
    """把速度指令裁到 PX4 EKF 给出的限值内（INFINITY=不限制）。"""
    v = np.asarray(v, float)
    if math.isfinite(vxy_max):
        vh = float(np.linalg.norm(v[:2]))
        if vh > vxy_max and vh > 1e-9:
            v[:2] *= vxy_max / vh
    if math.isfinite(vz_max):
        v[2] = float(np.clip(v[2], -vz_max, vz_max))
    return v


# --------------------------------------------------------------- 软围栏
def fence_velocity(v_sp, pos_world, *, geofence_xy: float, pullback_k: float,
                   pullback_speed: float, alt_floor: float,
                   geofence_alt: float) -> np.ndarray:
    """软围栏（世界系 NED）：在任务速度上**剔除继续向外**的分量，越界时叠加回拉。

    相比"直接替换为纯回拉速度"，保留沿围栏的切向机动，避免安全层与任务互相顶。
    """
    v = np.asarray(v_sp, float).copy()
    for i in (0, 1):
        p = float(pos_world[i])
        if p > geofence_xy:
            v[i] = min(v[i], -pullback_k * (p - geofence_xy))       # 越界：强制向内
        elif p < -geofence_xy:
            v[i] = max(v[i], pullback_k * (-geofence_xy - p))
        elif p > 0.0:
            v[i] = min(v[i], 0.0)                                   # 界内：禁止继续向外
        elif p < 0.0:
            v[i] = max(v[i], 0.0)
    alt = -float(pos_world[2])
    if alt > geofence_alt:
        v[2] = max(v[2], pullback_k * (alt - geofence_alt))         # 下压
    elif alt < alt_floor:
        v[2] = min(v[2], -pullback_k * (alt_floor - alt))           # 上拉
    n = float(np.linalg.norm(v[:2]))
    if n > pullback_speed and n > 1e-9:
        v[:2] *= pullback_speed / n
    v[2] = float(np.clip(v[2], -pullback_speed, pullback_speed))
    return v


# --------------------------------------------------------------- 姿态安全滤波
def attitude_govern(v, last_vsp, *, roll: float, pitch: float, ang_rate: float,
                    tilt_soft: float, tilt_hard: float,
                    rate_soft: float, rate_hard: float,
                    accel_h_max: float, hz: float,
                    enable: bool = True, att_ok: bool = True):
    """姿态/角速率约束（安全滤波）：用当前倾角/角速率衰减水平速度指令。

    - 倾角越接近硬限，水平指令衰减越多（到硬限=0，让姿态控制器回正）；
    - 角速率高时同样衰减；
    - 水平指令的**变化率**（即指令加速度）限额，且姿态越差额度越小。
    仅作用于水平方向，不改垂直；返回 (v, scale)。
    """
    v = np.asarray(v, float).copy()
    if (not enable) or (not att_ok):
        return v, 1.0
    tilt = max(abs(roll), abs(pitch))
    s_tilt = scale01(tilt, tilt_soft, tilt_hard)
    s_rate = scale01(ang_rate, rate_soft, rate_hard)
    s = min(s_tilt, s_rate)
    if s < 1.0:
        v[:2] *= s
    # 水平指令加速度限额（姿态越差额度越小）
    if accel_h_max > 0.0 and last_vsp is not None:
        dv = v[:2] - np.asarray(last_vsp, float)[:2]
        n = float(np.linalg.norm(dv))
        max_dv = accel_h_max / max(hz, 1e-6) * max(s_tilt, 0.15)
        if n > max_dv and n > 1e-9:
            v[:2] = np.asarray(last_vsp, float)[:2] + dv * (max_dv / n)
    return v, s


# --------------------------------------------------------------- 传感器健康
def sensor_health_reasons(*, watchdog_enable: bool, reset_counters,
                          lpos_valid: bool, dead_reckoning: bool,
                          heading_good: bool, heading_check: bool,
                          eph: float, epv: float,
                          eph_max: float, epv_max: float,
                          reset_seen_t: Optional[float], now: float,
                          reset_hold_s: float) -> list:
    """基于 PX4 已有 EKF 字段的健康/一致性约束（返回异常原因列表）。"""
    if (not watchdog_enable) or reset_counters is None:
        return []
    out = []
    if not lpos_valid:
        out.append('lpos_invalid')
    if dead_reckoning:
        out.append('dead_reckoning')
    if not heading_good and heading_check:
        out.append('heading_good_for_control=false')
    if eph > eph_max:
        out.append(f'eph={eph:.2f}>{eph_max:.2f}')
    if epv > epv_max:
        out.append(f'epv={epv:.2f}>{epv_max:.2f}')
    if reset_seen_t is not None and (now - reset_seen_t) < reset_hold_s:
        out.append('estimator_reset')
    return out


# --------------------------------------------------------------- 分级状态机
class SafetyDecision(NamedTuple):
    state: str                    # OK | HOLD | PULLBACK | LAND | KILL
    reason: str
    action: str                   # 'none' | 'kill' | 'land'
    bad_since: Optional[float]
    hold_since: Optional[float]
    event: str                    # '' | 'recover' | 'pullback' | 'hold' | 'hold_critical'


def safety_decision(now: float, state: str, reason: str,
                    bad_since: Optional[float], hold_since: Optional[float], *,
                    armed: bool, killed: bool, landing: bool, safety_lock: bool,
                    att_ok: bool, roll: float, pitch: float, safety_tilt_max: float,
                    safety_kill_hold_s: float, safety_auto_kill: bool,
                    safety_hold_escalate: str, safety_hold_timeout: float,
                    pos_world: Sequence[float], safety_geofence_xy: float,
                    safety_pullback_clear: float, alt_floor: float,
                    safety_geofence_alt: float, safety_pullback_enable: bool,
                    stale_reason: str, health_reasons: Sequence[str],
                    sensor_constraints_enable: bool) -> SafetyDecision:
    """分级安全响应：OK → (HOLD | PULLBACK) → LAND → KILL（纯函数，不改外部状态）。

    检测：姿态超限(临界) / 位置越界(回拉) / 位置状态超时(悬停)。
    临界异常持续 `safety_kill_hold_s`：`safety_auto_kill=True` 则飞行终止，否则按
    `safety_hold_escalate` 升级（none=保持 HOLD，land=降落）。
    """
    if not safety_lock or killed or landing:
        return SafetyDecision(state, reason, 'none', bad_since, hold_since, '')
    if not armed:
        # 未解锁：复位到 OK，清计时器（不刷日志）
        return SafetyDecision('OK', '', 'none', None, None, '')

    reasons = []
    critical = False
    if att_ok and (abs(roll) > safety_tilt_max or abs(pitch) > safety_tilt_max):
        reasons.append(f'tilt(r={math.degrees(roll):.0f},p={math.degrees(pitch):.0f})')
        critical = True
    _clr = safety_pullback_clear if state == 'PULLBACK' else 0.0
    out_xy = (abs(float(pos_world[0])) > safety_geofence_xy - _clr
              or abs(float(pos_world[1])) > safety_geofence_xy - _clr)
    alt = -float(pos_world[2])
    out_hi = alt > safety_geofence_alt - _clr
    out_lo = alt < alt_floor + _clr
    if out_xy or out_hi or out_lo:
        reasons.append(f'geofence(xy={np.round(pos_world[:2], 1)},alt={alt:.1f})')
    if stale_reason:
        reasons.append(stale_reason)
    health_bad = False
    if sensor_constraints_enable and health_reasons:
        reasons.extend(health_reasons)
        health_bad = True

    if not reasons:
        return SafetyDecision('OK', '', 'none', None, None,
                              'recover' if state != 'OK' else '')

    msg = '; '.join(reasons)

    if critical:
        if bad_since is None:
            bad_since = now
        if (now - bad_since) >= safety_kill_hold_s:
            bad_since = now                 # 避免刷屏
            if safety_auto_kill:
                return SafetyDecision(state, msg, 'kill', bad_since, hold_since, '')
            if safety_hold_escalate == 'land':
                return SafetyDecision(state, msg, 'land', bad_since, hold_since, '')
            return SafetyDecision('HOLD', msg, 'none', bad_since, hold_since,
                                  'hold_critical')
        return SafetyDecision('HOLD', msg, 'none', bad_since, hold_since, '')

    bad_since = None
    if ((out_xy or out_hi or out_lo) and safety_pullback_enable
            and not stale_reason and not health_bad):
        ev = 'pullback' if state != 'PULLBACK' else ''
        return SafetyDecision('PULLBACK', msg, 'none', bad_since, None, ev)
    if hold_since is None:
        hold_since = now
        ev = 'hold'
    else:
        ev = ''
    if (safety_hold_escalate == 'land'
            and (now - hold_since) >= safety_hold_timeout):
        return SafetyDecision(state, msg, 'land', bad_since, hold_since, '')
    return SafetyDecision('HOLD', msg, 'none', bad_since, hold_since, ev)


# --------------------------------------------------------------- 自测
def _selftest() -> bool:
    """最小自检（完整用例见 `tools/test_safety_logic.py`）。"""
    ok = True

    def chk(name, cond):
        nonlocal ok
        print(f'  {"✅" if cond else "❌"} {name}')
        ok = ok and bool(cond)

    chk('scale01 soft/hard', scale01(0.0, 1.0, 2.0) == 1.0
        and scale01(3.0, 1.0, 2.0) == 0.0
        and abs(scale01(1.5, 1.0, 2.0) - 0.5) < 1e-12)
    # 未解锁 → OK
    d = safety_decision(0.0, 'HOLD', 'x', 0.0, 0.0, armed=False, killed=False,
                        landing=False, safety_lock=True, att_ok=True, roll=0.0,
                        pitch=0.0, safety_tilt_max=1.0, safety_kill_hold_s=0.8,
                        safety_auto_kill=False, safety_hold_escalate='none',
                        safety_hold_timeout=8.0, pos_world=[0, 0, -3],
                        safety_geofence_xy=50.0, safety_pullback_clear=0.15,
                        alt_floor=-1.0, safety_geofence_alt=30.0,
                        safety_pullback_enable=True, stale_reason='',
                        health_reasons=[], sensor_constraints_enable=True)
    chk('未解锁复位 OK', d.state == 'OK' and d.bad_since is None)
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
