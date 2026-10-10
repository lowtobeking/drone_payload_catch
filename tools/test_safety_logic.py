#!/usr/bin/env python3
"""安全层纯逻辑（safety_logic.py）离线自检——无需 ROS/PX4/Gazebo。

    python3 tools/test_safety_logic.py

这些逻辑此前只活在 `px4_iface.py` 里、**只能在 SITL 中事后看日志**；抽出后
可离线钉死：围栏不顶任务、姿态滤波不误伤、传感器看门狗自愈、分级状态机
OK/HOLD/PULLBACK/LAND/KILL 的转换与升级。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import safety_logic as sl   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def decide(now, state='OK', reason='', bad_since=None, hold_since=None, **kw):
    """带默认参数地调用纯状态机，便于逐用例覆盖。"""
    a = dict(armed=True, killed=False, landing=False, safety_lock=True,
             att_ok=True, roll=0.0, pitch=0.0,
             safety_tilt_max=math.radians(60.0),
             safety_kill_hold_s=0.8, safety_auto_kill=False,
             safety_hold_escalate='none', safety_hold_timeout=8.0,
             pos_world=[0.0, 0.0, -3.0], safety_geofence_xy=50.0,
             safety_pullback_clear=0.15, alt_floor=-1.0,
             safety_geofence_alt=30.0, safety_pullback_enable=True,
             stale_reason='', health_reasons=[], sensor_constraints_enable=True)
    a.update(kw)
    return sl.safety_decision(now, state, reason, bad_since, hold_since, **a)


print('=== scale01 ===')
check('x≤soft→1', sl.scale01(0.0, 1.0, 2.0) == 1.0)
check('x≥hard→0', sl.scale01(3.0, 1.0, 2.0) == 0.0)
check('中点→0.5', abs(sl.scale01(1.5, 1.0, 2.0) - 0.5) < 1e-12)
check('hard≤soft 退化', sl.scale01(1.0, 2.0, 2.0) == 1.0 and sl.scale01(3.0, 2.0, 2.0) == 0.0)


print('\n=== fence_velocity（软围栏） ===')
v = sl.fence_velocity([1.0, 0.0, 0.0], [5.0, 0, -3], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('界内正向：禁止继续向外 → v_x=0', v[0] == 0.0, f'{v}')
v = sl.fence_velocity([-1.0, 0.0, 0.0], [5.0, 0, -3], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('界内反向：允许向内不变', v[0] == -1.0, f'{v}')
v = sl.fence_velocity([0.0, 0.0, 0.0], [-11.0, 0, -3], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('越界(−x)：强制向回拉 v_x>0', v[0] > 0, f'{v}')
v = sl.fence_velocity([0.0, 0.5, 0.0], [10.1, -1.0, -3], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('越界(+x) + 切向向内：保留 v_y', v[0] < 0 and abs(v[1] - 0.5) < 1e-12, f'{v}')
v = sl.fence_velocity([0.0, 0.0, 0.0], [0, 0, -31], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('超高：下压 v_z>0', v[2] > 0, f'{v}')
v = sl.fence_velocity([0.0, 0.0, 0.0], [0, 0, -0.05], geofence_xy=10, pullback_k=2,
                      pullback_speed=2, alt_floor=0.1, geofence_alt=30)
check('过低：上拉 v_z<0', v[2] < 0, f'{v}')
v = sl.fence_velocity([0.0, 10.0, 0.0], [-11.0, 0, -3], geofence_xy=10, pullback_k=1,
                      pullback_speed=2, alt_floor=-1, geofence_alt=30)
check('回拉速度上限 |v_xy|≤2', abs(np.linalg.norm(v[:2]) - 2.0) < 1e-9,
      f'|v_xy|={np.linalg.norm(v[:2]):.4f}')


print('\n=== attitude_govern（姿态安全滤波） ===')
v, s = sl.attitude_govern([1.0, 0, 0], None, roll=0.0, pitch=0.0, ang_rate=0.0,
                          tilt_soft=math.radians(25), tilt_hard=math.radians(40),
                          rate_soft=math.radians(150), rate_hard=math.radians(300),
                          accel_h_max=0.0, hz=50)
check('倾角/角速率正常：不衰减', np.allclose(v, [1, 0, 0]) and s == 1.0)
v, s = sl.attitude_govern([1.0, 0, 0], None, roll=math.radians(40), pitch=0.0,
                          ang_rate=0.0, tilt_soft=math.radians(25),
                          tilt_hard=math.radians(40), rate_soft=math.radians(150),
                          rate_hard=math.radians(300), accel_h_max=0.0, hz=50)
check('倾角到硬限：水平清零', abs(v[0]) < 1e-12 and s == 0.0, f'v={v} s={s:.3f}')
v, s = sl.attitude_govern([1.0, 0, 0], None, roll=0.0, pitch=0.0,
                          ang_rate=math.radians(300), tilt_soft=math.radians(25),
                          tilt_hard=math.radians(40), rate_soft=math.radians(150),
                          rate_hard=math.radians(300), accel_h_max=0.0, hz=50)
check('角速率到硬限：水平清零', abs(v[0]) < 1e-12 and s == 0.0)
v, s = sl.attitude_govern([1.0, 0, 0], None, roll=0.0, pitch=0.0, ang_rate=0.0,
                          tilt_soft=math.radians(25), tilt_hard=math.radians(40),
                          rate_soft=math.radians(150), rate_hard=math.radians(300),
                          accel_h_max=5.0, hz=50, enable=False)
check('总开关关：不衰减', np.allclose(v, [1, 0, 0]) and s == 1.0)
v, s = sl.attitude_govern([1.0, 0, 0], None, roll=0.0, pitch=0.0, ang_rate=0.0,
                          tilt_soft=math.radians(25), tilt_hard=math.radians(40),
                          rate_soft=math.radians(150), rate_hard=math.radians(300),
                          accel_h_max=5.0, hz=50, att_ok=False)
check('姿态无效：不衰减（避免误伤）', np.allclose(v, [1, 0, 0]) and s == 1.0)
v, s = sl.attitude_govern([10.0, 0, 0], [0.0, 0, 0], roll=0.0, pitch=0.0, ang_rate=0.0,
                          tilt_soft=math.radians(25), tilt_hard=math.radians(40),
                          rate_soft=math.radians(150), rate_hard=math.radians(300),
                          accel_h_max=5.0, hz=50)
check('水平指令加速度限额：|Δv|≤a_max/hz', abs(v[0] - 0.1) < 1e-9, f'v_x={v[0]:.4f}')
v, s = sl.attitude_govern([1.0, 2.0, 3.0], None, roll=math.radians(40), pitch=0.0,
                          ang_rate=0.0, tilt_soft=math.radians(25),
                          tilt_hard=math.radians(40), rate_soft=math.radians(150),
                          rate_hard=math.radians(300), accel_h_max=0.0, hz=50)
check('垂直不受姿态滤波影响', v[2] == 3.0, f'v={v}')


print('\n=== sensor_health_reasons（EKF 看门狗） ===')
base = dict(watchdog_enable=True, reset_counters=(0, 0, 0, 0, 0),
            lpos_valid=True, dead_reckoning=False, heading_good=True,
            heading_check=False, eph=0.1, epv=0.1, eph_max=0.5, epv_max=0.5,
            reset_seen_t=None, now=10.0, reset_hold_s=1.0)
check('全健康 → []', sl.sensor_health_reasons(**base) == [])
check('看门狗关 → []', sl.sensor_health_reasons(**{**base, 'watchdog_enable': False}) == [])
check('reset_counters None → []',
      sl.sensor_health_reasons(**{**base, 'reset_counters': None}) == [])
check('lpos 无效', 'lpos_invalid' in sl.sensor_health_reasons(**{**base, 'lpos_valid': False}))
check('dead_reckoning',
      'dead_reckoning' in sl.sensor_health_reasons(**{**base, 'dead_reckoning': True}))
check('heading=false 且开启检查',
      any('heading' in r for r in sl.sensor_health_reasons(
          **{**base, 'heading_good': False, 'heading_check': True})))
check('heading=false 但未开启检查 → 不报',
      sl.sensor_health_reasons(**{**base, 'heading_good': False, 'heading_check': False}) == [])
check('eph 超限', any('eph' in r for r in sl.sensor_health_reasons(**{**base, 'eph': 0.9})))
check('epv 超限', any('epv' in r for r in sl.sensor_health_reasons(**{**base, 'epv': 0.9})))
check('估计器跳变窗口内报',
      'estimator_reset' in sl.sensor_health_reasons(
          **{**base, 'reset_seen_t': 9.5, 'now': 10.0, 'reset_hold_s': 1.0}))
check('跳变窗口外不报',
      'estimator_reset' not in sl.sensor_health_reasons(
          **{**base, 'reset_seen_t': 8.0, 'now': 10.0, 'reset_hold_s': 1.0}))


print('\n=== safety_decision（分级状态机） ===')
d = decide(0.0, state='HOLD', reason='x', bad_since=0.0, hold_since=0.0, armed=False)
check('未解锁 → 复位 OK、清计时器', d.state == 'OK' and d.bad_since is None
      and d.hold_since is None and d.action == 'none')
d = decide(0.0, state='HOLD', reason='r', killed=True)
check('已 kill → 不改状态、无动作', d.state == 'HOLD' and d.action == 'none')
d = decide(0.0, state='HOLD', reason='r', safety_lock=False)
check('安全监督关 → 不改状态', d.state == 'HOLD' and d.action == 'none')
d = decide(0.0)
check('全部正常 → OK', d.state == 'OK')
d = decide(0.0, state='HOLD', reason='x', hold_since=1.0)
check('恢复正常 → OK 且 event=recover', d.state == 'OK' and d.event == 'recover')

# 姿态临界
roll_crit = math.radians(70.0)
d = decide(0.0, roll=roll_crit)
check('姿态超限首拍 → HOLD，记录 bad_since', d.state == 'HOLD' and d.bad_since == 0.0
      and d.action == 'none')
d = decide(0.5, state='HOLD', bad_since=0.0, roll=roll_crit)
check('未达 kill_hold_s → 仍 HOLD（不升级）', d.state == 'HOLD' and d.action == 'none')
d = decide(1.0, state='HOLD', bad_since=0.0, roll=roll_crit, safety_auto_kill=True)
check('持续超限 + auto_kill → action=kill', d.action == 'kill')
d = decide(1.0, state='HOLD', bad_since=0.0, roll=roll_crit,
           safety_hold_escalate='land')
check('持续超限 + escalate=land → action=land', d.action == 'land')
d = decide(1.0, state='HOLD', bad_since=0.0, roll=roll_crit)
check('持续超限 + none → HOLD + event=hold_critical',
      d.state == 'HOLD' and d.action == 'none' and d.event == 'hold_critical')

# 围栏 / 回拉
d = decide(0.0, pos_world=[51.0, 0, -3])
check('越界 → PULLBACK + event=pullback', d.state == 'PULLBACK' and d.event == 'pullback')
d = decide(1.0, state='PULLBACK', pos_world=[51.0, 0, -3])
check('已在 PULLBACK → 不再重复打日志', d.state == 'PULLBACK' and d.event == '')
d = decide(0.0, state='PULLBACK', pos_world=[49.9, 0, -3])
check('回拉滞环：49.9 > gx−clear=49.85 仍 PULLBACK', d.state == 'PULLBACK')
d = decide(0.0, pos_world=[49.9, 0, -3])
check('OK 态下 49.9 < gx=50 → 不越界', d.state == 'OK')
d = decide(0.0, pos_world=[51.0, 0, -3], safety_pullback_enable=False)
check('回拉关闭：越界也只 HOLD', d.state == 'HOLD')
d = decide(0.0, pos_world=[51.0, 0, -3], stale_reason='pos_stale=3.0s')
check('越界 + 状态超时 → HOLD（不回拉）', d.state == 'HOLD')
d = decide(0.0, pos_world=[51.0, 0, -3], health_reasons=['dead_reckoning'])
check('越界 + 健康异常 → HOLD（不回拉）', d.state == 'HOLD')

# 状态超时
d = decide(0.0, stale_reason='pos_stale=5.0s')
check('仅状态超时 → HOLD', d.state == 'HOLD' and d.event == 'hold')
d = decide(1.0, state='HOLD', hold_since=0.0, stale_reason='pos_stale=5.0s')
check('HOLD 持续中 → 不重复打日志', d.event == '')

# HOLD 升级 LAND
d = decide(9.0, state='HOLD', hold_since=0.0, stale_reason='pos_stale=5.0s',
           safety_hold_escalate='land', safety_hold_timeout=8.0)
check('HOLD 超时 + escalate=land → action=land', d.action == 'land')


print('\n=== 电池保护 ===')
check('低电 → Land', decide(0.0, battery_land_reason='battery warning=2').action == 'land')
check('紧急低电 → Land', decide(0.0, state='HOLD', battery_land_reason='battery warning=3').action == 'land')
check('无低电 → 无动作', decide(0.0).action == 'none')
check('land_reason warning≥2', sl.battery_land_reason(connected=True, warning=2, remaining=0.5) != '')
check('land_reason remaining<crit', sl.battery_land_reason(connected=True, warning=0, remaining=0.05) != '')
check('land_reason 正常→空', sl.battery_land_reason(connected=True, warning=0, remaining=0.9) == '')
check('land_reason 未连接→空', sl.battery_land_reason(connected=False, warning=3, remaining=0.0) == '')
check('warn_reason low', sl.battery_warn_reason(connected=True, warning=1, remaining=0.5) != '')

print('\n=== clip_to_estimator_limits ===')
v = sl.clip_to_estimator_limits([10.0, 0.0, 5.0], 2.0, 1.0)
check('水平裁到 vxy_max', abs(np.linalg.norm(v[:2]) - 2.0) < 1e-9)
check('垂直裁到 ±vz_max', v[2] == 1.0)
v = sl.clip_to_estimator_limits([10.0, 0.0, 5.0], float('inf'), float('inf'))
check('INFINITY=不限制', np.allclose(v, [10, 0, 5]))

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_safety_logic 全部通过')
