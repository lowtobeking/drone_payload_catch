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
                       'fs_bad_mag_y': False, 'fs_bad_mag_z': False, 'fs_bad_hdg': False},
        n_imu=50, imu_present=True, imu_error=False, vibration_max=0.5,
        n_gps=50, gps_max_eph=0.9, gps_max_epv=1.2, gps_min_sats=25)


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
check('offboard 信号丢失 → fail',
      status(evaluate(s), 'failsafe: offboard 信号丢失') == 'fail')
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

print('\n=== IMU ===')
s = healthy(); s.imu_present = False
check('IMU device_id=0 → fail', status(evaluate(s), 'IMU 在线(device_id)') == 'fail')
s = healthy(); s.imu_error = True
check('IMU error_count → 仅 warn',
      status(evaluate(s), 'IMU 无 error_count') == 'warn' and not has_fail(evaluate(s)))
s = healthy(); s.vibration_max = 10.0
check('振动超限 → 仅 warn', status(evaluate(s), '振动在阈值内') == 'warn')

print('\n=== GPS（--gps 严格门）===')
s = healthy()
check('默认不判 GPS → 无 fail', not has_fail(evaluate(s)))
check('--gps + 无 GPS → fail', status(evaluate(Snapshot(n_lpos=10, xy_valid_frac=1,
      z_valid_frac=1, v_valid_frac=1), require_gps=True), 'sensor_gps') == 'fail')
s = healthy(); s.gps_max_eph = 2.0
check('GPS eph 2.0 > 1.5 → fail', status(evaluate(s, require_gps=True),
      'GPS eph ≤ 1.5') == 'fail')
s = healthy(); s.gps_min_sats = 10
check('sats 10 < 20 → fail', status(evaluate(s, require_gps=True),
      'GPS sats ≥ 20') == 'fail')
check('GPS 达标 → 无 fail', not has_fail(evaluate(healthy(), require_gps=True)))

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_live_probe 全部通过')
