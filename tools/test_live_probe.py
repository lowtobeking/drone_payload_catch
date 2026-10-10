#!/usr/bin/env python3
"""运行时探测判定逻辑（live_probe.evaluate）离线单测——构造合成 Snapshot，无需 ROS。

    python3 tools/test_live_probe.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.live_probe import Snapshot, evaluate   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def status(res, name):
    for r in res:
        if r['name'] == name:
            return r['status']
    return None


def has_fail(res):
    return any(r['status'] == 'fail' for r in res)


def healthy():
    return Snapshot(
        n_lpos=150, xy_valid_frac=1.0, z_valid_frac=1.0, v_valid_frac=1.0,
        dead_reckoning_frac=0.0, heading_good_frac=1.0, max_eph=0.15, max_epv=0.20,
        n_failsafe=50, failsafe={f: False for f in (
            'local_position_invalid', 'local_velocity_invalid', 'attitude_invalid',
            'offboard_control_signal_lost', 'geofence_breached', 'fd_critical_failure',
            'battery_unhealthy')},
        n_est=50, est={'cs_tilt_align': True, 'cs_yaw_align': True, 'cs_mag': True,
                       'cs_mag_hdg': False, 'cs_mag_3d': True, 'cs_mag_fault': False,
                       'cs_mag_field_disturbed': False, 'fs_bad_mag_x': False,
                       'fs_bad_mag_y': False, 'fs_bad_mag_z': False, 'fs_bad_hdg': False,
                       'fs_bad_acc_vertical': False, 'fs_bad_acc_clipping': False},
        n_gps=50, gps_max_eph=0.9, gps_max_epv=1.2, gps_min_sats=25,
        n_batt=50, batt_connected=True, batt_warning_max=0, batt_remaining_min=0.8)


print('=== 健康 → 无 fail ===')
r = evaluate(healthy())
check('健康快照无 fail', not has_fail(r), f'{len(r)} 项')

print('\n=== EKF 有效性 ===')
r = evaluate(Snapshot())                                   # 什么都没收到
check('lpos 未收到 → fail', status(r, 'vehicle_local_position') == 'fail')
s = healthy(); s.xy_valid_frac = 0.5
check('xy_valid 50% → fail', status(evaluate(s), 'xy_valid 持续') == 'fail')
s = healthy(); s.dead_reckoning_frac = 0.5
check('dead_reckoning 50% → fail', status(evaluate(s), '非 dead_reckoning') == 'fail')
s = healthy(); s.max_eph = 0.9
check('eph 0.9 > 0.5 → fail', status(evaluate(s), 'eph ≤ 0.5') == 'fail')
s = healthy(); s.max_epv = 0.9
check('epv 0.9 > 0.5 → fail', status(evaluate(s), 'epv ≤ 0.5') == 'fail')

print('\n=== failsafe flags ===')
s = healthy(); s.failsafe['local_position_invalid'] = True
check('local_position_invalid → fail',
      status(evaluate(s), 'failsafe: 本地位置无效') == 'fail')
s = healthy(); s.failsafe['offboard_control_signal_lost'] = True
r = evaluate(s)
check('offboard 信号丢失 → 仅 warn（起飞前正常）',
      status(r, 'failsafe: offboard 信号') == 'warn' and not has_fail(r))
s = healthy(); s.failsafe['battery_unhealthy'] = True
r = evaluate(s)
check('battery_unhealthy → 仅 warn', status(r, 'battery_unhealthy') == 'warn' and not has_fail(r))

print('\n=== estimator / 磁罗盘 ===')
s = healthy(); s.est['cs_mag_fault'] = True
check('磁故障 → fail', status(evaluate(s), '无磁故障') == 'fail')
s = healthy(); s.est['fs_bad_hdg'] = True
check('磁融合数值错误 → fail', status(evaluate(s), '无磁融合数值错误') == 'fail')
s = healthy()
for k in ('cs_mag', 'cs_mag_hdg', 'cs_mag_3d'):
    s.est[k] = False
r = evaluate(s)
check('未融合磁 → 仅 warn', status(r, '磁罗盘在线') == 'warn' and not has_fail(r))
s = healthy(); s.est['cs_tilt_align'] = False
check('tilt 未对齐 → fail', status(evaluate(s), 'tilt/yaw 对齐') == 'fail')
r = evaluate(Snapshot(n_lpos=10, xy_valid_frac=1, z_valid_frac=1, v_valid_frac=1))
check('estimator 未收到 → warn', status(r, 'estimator_status_flags') == 'warn')

print('\n=== 加计故障（estimator_status_flags）===')
s = healthy(); s.est['fs_bad_acc_vertical'] = True
check('加计垂直故障 → fail',
      status(evaluate(s), '无加计故障(垂直/削波)') == 'fail')
s = healthy(); s.est['fs_bad_acc_clipping'] = True
check('加计削波 → fail',
      status(evaluate(s), '无加计故障(垂直/削波)') == 'fail')

print('\n=== GPS（--gps 严格门）===')
s = healthy()
check('默认不判 GPS → 无 fail', not has_fail(evaluate(s)))
check('--gps + 无 GPS → fail', status(evaluate(Snapshot(n_lpos=10, xy_valid_frac=1,
      z_valid_frac=1, v_valid_frac=1), require_gps=True), 'vehicle_gps_position') == 'fail')
s = healthy(); s.gps_max_eph = 2.0
check('GPS eph 2.0 > 1.5 → fail', status(evaluate(s, require_gps=True),
      'GPS eph ≤ 1.5') == 'fail')
s = healthy(); s.gps_min_sats = 10
check('sats 10 < 20 → fail', status(evaluate(s, require_gps=True),
      'GPS sats ≥ 20') == 'fail')
check('GPS 达标 → 无 fail', not has_fail(evaluate(healthy(), require_gps=True)))
check('阈值可调：sats_min=8 时 10 星通过',
      not has_fail(evaluate(healthy(), require_gps=True, gps_sats_min=8)))

print('\n=== 电池（起飞前）===')
check('电池正常 → 无 fail', not has_fail(evaluate(healthy())))
s = healthy(); s.batt_warning_max = 2
check('warning≥2 → fail', status(evaluate(s), '电池 warning ≤ low') == 'fail')
s = healthy(); s.batt_warning_max = 1
check('warning=1 → warn', status(evaluate(s), '电池 warning ≤ low') == 'warn')
s = healthy(); s.batt_remaining_min = 0.05
check('remaining<0.07 → fail', status(evaluate(s), '电池 remaining ≥ 0.15') == 'fail')
s = healthy(); s.batt_connected = False
check('未连接 → warn', status(evaluate(s), '电池已连接') == 'warn')

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_live_probe 全部通过')
