"""Fail-closed takeoff health checks shared by CLI and flight nodes; no ROS."""
from __future__ import annotations

import json
import copy
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

BATTERY_MIN = 0.40
WINDOW_S = 3.0
APPROVAL_TTL_S = 60.0
EPH_MAX, EPV_MAX = 0.50, 0.50
GPS_EPH_MAX, GPS_EPV_MAX, GPS_SATS_MIN = 1.5, 2.5, 20
MAX_AGE = dict(lpos=1.0, att=1.0, status=2.0, failsafe=2.0, est=2.0,
               batt=3.0, gps=2.0, rc=0.5)
FAILSAFE_FATAL = [
    ('local_position_invalid', 'failsafe: 本地位置无效'),
    ('local_velocity_invalid', 'failsafe: 本地速度无效'),
    ('attitude_invalid', 'failsafe: 姿态无效'),
    ('geofence_breached', 'failsafe: 越界'),
    ('fd_critical_failure', 'failsafe: 临界故障'),
    ('battery_unhealthy', 'battery_unhealthy'),
]
EST_BAD = ('cs_mag_fault', 'cs_mag_field_disturbed', 'fs_bad_mag_x',
           'fs_bad_mag_y', 'fs_bad_mag_z', 'fs_bad_hdg',
           'fs_bad_acc_vertical', 'fs_bad_acc_clipping')


@dataclass
class Snapshot:
    n_lpos: int = 0
    xy_valid_frac: float = 0.0
    z_valid_frac: float = 0.0
    v_valid_frac: float = 0.0
    dead_reckoning_frac: float = 0.0
    heading_good_frac: float = 0.0
    max_eph: float = 0.0
    max_epv: float = 0.0
    n_failsafe: int = 0
    failsafe: dict = field(default_factory=dict)
    n_est: int = 0
    est: dict = field(default_factory=dict)
    n_gps: int = 0
    gps_max_eph: float = 0.0
    gps_max_epv: float = 0.0
    gps_min_sats: int = 999
    n_batt: int = 0
    batt_connected: bool = False
    batt_warning_max: int = 0
    batt_remaining_min: float = math.nan
    n_status: int = 0
    checks_pass: bool = False
    n_att: int = 0
    att_valid: bool = False
    ages: dict = field(default_factory=dict)
    invalid_data: tuple = ()
    n_rc: int = 0
    rc_valid: bool = False


def evaluate(snap: Snapshot, *, require_gps=False, require_rc=False, eph_max=EPH_MAX,
             epv_max=EPV_MAX, gps_eph_max=GPS_EPH_MAX,
             gps_epv_max=GPS_EPV_MAX, gps_sats_min=GPS_SATS_MIN):
    """Unknown/missing/stale health data always blocks takeoff."""
    out = []

    def add(name, good, detail='', warning=False):
        out.append(dict(name=name, status='ok' if good else ('warn' if warning else 'fail'),
                        detail=detail))

    required = dict(lpos='vehicle_local_position', att='vehicle_attitude',
                    status='vehicle_status', failsafe='failsafe_flags',
                    est='estimator_status_flags', batt='battery_status')
    if require_gps:
        required['gps'] = 'vehicle_gps_position'
    if require_rc:
        required['rc'] = 'manual_control_setpoint(RC)'
    for key, label in required.items():
        count = getattr(snap, 'n_' + key)
        add(label, count >= 2, f'{count} 帧（至少两帧且源时间戳递增）')
        age = snap.ages.get(key, math.inf)
        add(label + ' 新鲜', math.isfinite(age) and 0 <= age <= MAX_AGE[key],
            f'age={age:.2f}s')
    add('关键数据有限且完整', not snap.invalid_data, ','.join(snap.invalid_data))
    add('PX4 pre_flight_checks_pass', snap.checks_pass)
    add('姿态有效', snap.att_valid)
    add('xy_valid 持续', snap.xy_valid_frac == 1.0)
    add('z_valid 持续', snap.z_valid_frac == 1.0)
    add('v_xy/z_valid 持续', snap.v_valid_frac == 1.0)
    add('非 dead_reckoning', snap.dead_reckoning_frac == 0.0)
    add('heading_good_for_control', snap.heading_good_frac == 1.0)
    add(f'eph ≤ {eph_max}', math.isfinite(snap.max_eph) and 0 <= snap.max_eph <= eph_max)
    add(f'epv ≤ {epv_max}', math.isfinite(snap.max_epv) and 0 <= snap.max_epv <= epv_max)
    for flag, label in FAILSAFE_FATAL:
        add(label, flag in snap.failsafe and not snap.failsafe[flag])
    # No Offboard heartbeats before authorization: signal_lost is expected here.
    add('failsafe: offboard 信号', not snap.failsafe.get('offboard_control_signal_lost', True),
        '起飞前未发送 Offboard 心跳时可为 true', warning=True)
    add('tilt/yaw 对齐', bool(snap.est.get('cs_tilt_align') and snap.est.get('cs_yaw_align')))
    for flag in EST_BAD:
        add('estimator: ' + flag, flag in snap.est and not snap.est[flag])
    if require_gps:
        add(f'GPS eph ≤ {gps_eph_max}', math.isfinite(snap.gps_max_eph)
            and 0 <= snap.gps_max_eph <= gps_eph_max)
        add(f'GPS epv ≤ {gps_epv_max}', math.isfinite(snap.gps_max_epv)
            and 0 <= snap.gps_max_epv <= gps_epv_max)
        add(f'GPS sats ≥ {gps_sats_min}', snap.gps_min_sats >= gps_sats_min)
    add('电池已连接', snap.batt_connected)
    add('电池 warning = 0', snap.batt_warning_max == 0)
    if require_rc:
        add('独立 RC 信号有效', snap.rc_valid)
    add('电池 remaining ≥ 0.40', math.isfinite(snap.batt_remaining_min)
        and BATTERY_MIN <= snap.batt_remaining_min <= 1.0,
        f'remaining={snap.batt_remaining_min:.3f}')
    return out


class HealthAccumulator:
    """Bounded rolling window; duplicate/old PX4 samples cannot refresh health."""
    def __init__(self, window_s=WINDOW_S):
        self.window_s = window_s
        self.samples = {key: deque() for key in MAX_AGE}
        self.last_stamp = {}

    def record(self, key, msg, now):
        stamp = getattr(msg, 'timestamp', 0)
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            self.samples[key].append((now, None))
        elif stamp > self.last_stamp.get(key, 0):
            self.last_stamp[key] = stamp
            # Snapshot callback fields now; ROS may recycle/mutate message objects.
            self.samples[key].append((now, copy.deepcopy(msg)))
        self._trim(now)

    def _trim(self, now):
        for queue in self.samples.values():
            while queue and now - queue[0][0] > self.window_s:
                queue.popleft()

    def snapshot(self, now):
        self._trim(now)
        s = Snapshot()
        invalid = set()
        for key, queue in self.samples.items():
            setattr(s, 'n_' + key, len(queue))
            s.ages[key] = now - queue[-1][0] if queue else math.inf
            if any(m is None for _, m in queue):
                invalid.add(key)

        def values(key, name):
            vals = [getattr(m, name, None) for _, m in self.samples[key]]
            if not vals or any(v is None for v in vals):
                invalid.add(key + '.' + name)
            return vals

        def all_true(key, name):
            vals = values(key, name)
            return bool(vals) and all(v is True for v in vals)

        def bad_any(key, name):
            vals = values(key, name)
            return any(v is not False for v in vals)

        def numbers(key, name):
            vals = values(key, name)
            if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in vals):
                invalid.add(key + '.' + name)
                return [math.inf]
            return vals

        s.xy_valid_frac = float(all_true('lpos', 'xy_valid'))
        s.z_valid_frac = float(all_true('lpos', 'z_valid'))
        s.v_valid_frac = float(all_true('lpos', 'v_xy_valid') and all_true('lpos', 'v_z_valid'))
        s.heading_good_frac = float(all_true('lpos', 'heading_good_for_control'))
        s.dead_reckoning_frac = float(bad_any('lpos', 'dead_reckoning'))
        for name in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
            numbers('lpos', name)
        s.max_eph = max(numbers('lpos', 'eph'), default=math.inf)
        s.max_epv = max(numbers('lpos', 'epv'), default=math.inf)
        s.failsafe = {k: bad_any('failsafe', k) for k, _ in FAILSAFE_FATAL}
        s.failsafe['offboard_control_signal_lost'] = bad_any('failsafe', 'offboard_control_signal_lost')
        s.est = {k: bad_any('est', k) for k in EST_BAD}
        for name in ('cs_tilt_align', 'cs_yaw_align'):
            s.est[name] = all_true('est', name)
        s.checks_pass = all_true('status', 'pre_flight_checks_pass')
        s.batt_connected = all_true('batt', 'connected')
        s.batt_warning_max = max(numbers('batt', 'warning'), default=math.inf)
        s.batt_remaining_min = min(numbers('batt', 'remaining'), default=math.nan)
        s.att_valid = bool(self.samples['att'])
        for q in values('att', 'q'):
            try:
                norm2 = sum(float(x) ** 2 for x in q)
                if len(q) != 4 or not math.isfinite(norm2) or not 0.8 <= norm2 <= 1.2:
                    s.att_valid = False
                    invalid.add('att.q')
            except (TypeError, ValueError, OverflowError):
                s.att_valid = False
                invalid.add('att.q')
        # GPS is optional for indoor/SITL; absent GPS must not poison other checks.
        if self.samples['gps']:
            s.gps_max_eph = max(numbers('gps', 'eph'), default=math.inf)
            s.gps_max_epv = max(numbers('gps', 'epv'), default=math.inf)
            s.gps_min_sats = min(numbers('gps', 'satellites_used'), default=0)
        if self.samples['rc']:
            s.rc_valid = all_true('rc','valid') and all(
                getattr(m,'data_source',None) == 1 for _,m in self.samples['rc'])
        s.invalid_data = tuple(sorted(invalid))
        return s


def approval_reason(report, *, now=None, require_gps=True, require_rc=False):
    """Validate a short-lived, both-aircraft preflight receipt. Not a security token."""
    now = time.time() if now is None else now
    if not isinstance(report, dict) or report.get('schema') != 1 or report.get('passed') is not True:
        return '缺少有效的起飞前检查通过记录'
    at = report.get('checked_at')
    if not isinstance(at, (int, float)) or not math.isfinite(at) or not 0 <= now - at <= APPROVAL_TTL_S:
        return '起飞前检查记录过期／时间异常（有效期 60 秒）'
    if report.get('battery_min') != BATTERY_MIN or report.get('drone_ids') != [0, 1]:
        return '检查记录未满足双机／40% 电量要求'
    if require_gps and report.get('gps') is not True:
        return '室外起飞检查记录缺少 GPS 检查'
    if require_rc and report.get('rc') is not True:
        return '真机起飞检查记录缺少独立 RC 信号检查'
    if report.get('params_checked') != [0, 1]:
        return '两台飞控参数未全部检查通过'
    return ''


def read_approval(path, *, now=None, require_gps=True, require_rc=False):
    try:
        report = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError, TypeError):
        return '未提供可读取的 preflight_report；先运行强制双机自检'
    return approval_reason(report, now=now, require_gps=require_gps,require_rc=require_rc)


class TakeoffGate:
    """Require BOTH aircraft healthy continuously; no disable/battery override."""
    def __init__(self, require_gps=True, require_rc=False):
        self.require_gps = require_gps
        self.require_rc = require_rc
        self.health = {i: HealthAccumulator() for i in (0, 1)}
        self.good_since = None

    def check(self, now, approval_error=''):
        failures = [approval_error] if approval_error else []
        for i, acc in self.health.items():
            failures += [f'd{i}: {r["name"]}' for r in evaluate(
                acc.snapshot(now), require_gps=self.require_gps,
                require_rc=self.require_rc) if r['status'] == 'fail']
        if failures:
            self.good_since = None
            return False, '; '.join(failures)
        if self.good_since is None:
            self.good_since = now
        if now - self.good_since < WINDOW_S:
            return False, '双机健康连续观察中（3 秒）'
        return True, ''
