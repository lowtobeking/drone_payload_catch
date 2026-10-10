#!/usr/bin/env python3
"""故障注入矩阵判定逻辑（payload_catch/safety_matrix.py）离线单测——无需 ROS/SITL。

    python3 tools/test_safety_matrix.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.safety_matrix import SCENARIOS, Scenario, verdict   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


print('=== verdict 基本 ===')
sc = Scenario('t', expect_present=[r'STACK CAPTURED'], expect_absent=[r'collision_floor'])
check('命中 present + 无 absent → PASS', verdict('... STACK CAPTURED ...', sc)[0])
check('缺 present → FAIL', not verdict('nothing', sc)[0])
check('出现 absent → FAIL', not verdict('STACK CAPTURED\ncollision_floor', sc)[0])

print('\n=== 基线：容忍启动瞬态 estimator_reset，但不容忍其它安全事件 ===')
base = next(s for s in SCENARIOS if s.name == 'baseline_no_false_trigger')
log_ok = ('[b_node] SAFETY HOLD: estimator_reset\n'
          '[b_node] SAFETY 恢复 → OK\n'
          '[b_node] *** STACK CAPTURED *** horiz=0.01')
check('含瞬态 HOLD + 捕获 → PASS', verdict(log_ok, base)[0])
check('含 collision_floor → FAIL', not verdict(log_ok + '\ncollision_floor', base)[0])
check('含 SAFETY PULLBACK → FAIL', not verdict(log_ok + '\nSAFETY PULLBACK', base)[0])
check('含 Failsafe activated → FAIL',
      not verdict(log_ok + '\nFailsafe activated', base)[0])

print('\n=== 各诱发场景断言 ===')
geo = next(s for s in SCENARIOS if s.name == 'geofence_pullback')
check('geofence 需要 PULLBACK', verdict('SAFETY PULLBACK: geofence', geo)[0])
check('geofence 出现 failsafe → FAIL',
      not verdict('SAFETY PULLBACK\nFailsafe activated', geo)[0])
col = next(s for s in SCENARIOS if s.name == 'collision_floor_hold')
check('collision 需要 collision_floor HOLD',
      verdict('SAFETY HOLD: collision_floor(d=1.4)', col)[0])
peer = next(s for s in SCENARIOS if s.name == 'peer_loss_freeze_land')
check('peer_loss 需要冻结+降落',
      verdict('B: 失联 1.0s 未收到 A 状态 → 就地冻结（不追陈旧参考）\n'
              'B: 失联 4.0s → 安全悬停并降落', peer)[0])
check('peer_loss 只冻结未降落 → FAIL',
      not verdict('失联 1.0s → 就地冻结', peer)[0])

print('\n=== SCENARIOS 自洽 ===')
check('至少 5 个场景', len(SCENARIOS) >= 5)
check('每场景有 name/expect_present/why',
      all(s.name and s.expect_present and s.why for s in SCENARIOS))
check('含零误触发基线', any(s.name == 'baseline_no_false_trigger' for s in SCENARIOS))

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_safety_matrix 全部通过')
