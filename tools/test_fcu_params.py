#!/usr/bin/env python3
"""飞控安全参数判据（payload_catch/fcu_params.py）离线单测——合成参数，无需 ROS/MAVLink。

    python3 tools/test_fcu_params.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import fcu_params as F   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def status(issues, name):
    for i in issues:
        if i.name == name:
            return i.level
    return None


def has_fail(issues):
    return any(i.level == 'fail' for i in issues)


print('=== parse_param_dump 多格式 ===')
txt = ("MPC_THR_MIN 0.1200\n"
       "# comment\n"
       "1 1 MPC_THR_HOVER 0.3000\n"
       "COM_RCL_EXCEPT=0\n"
       "MPC_XY_VEL_MAX,2.0\n")
p = F.parse_param_dump(txt)
check('空格/等号/逗号/带序号四种格式', p.get('MPC_THR_MIN') == 0.12
      and p.get('MPC_THR_HOVER') == 0.30 and p.get('COM_RCL_EXCEPT') == 0
      and p.get('MPC_XY_VEL_MAX') == 2.0, str(p))

print('\n=== 关键关系 ===')
good = {'MPC_THR_MIN': 0.12, 'MPC_THR_HOVER': 0.30, 'COM_RCL_EXCEPT': 0,
        'MPC_XY_VEL_MAX': 2.0}
check('好参数（sitl）无 fail', not has_fail(F.check_params(good, 'sitl')))
check('THR_MIN=0.30=HOVER → fail',
      status(F.check_params({**good, 'MPC_THR_MIN': 0.30}), 'MPC_THR_MIN < MPC_THR_HOVER') == 'fail')
check('THR_MIN=0.28 余量不足 → fail',
      status(F.check_params({**good, 'MPC_THR_MIN': 0.28}), 'MPC_THR_MIN < MPC_THR_HOVER') == 'fail')
check('COM_RCL_EXCEPT=4 → fail',
      status(F.check_params({**good, 'COM_RCL_EXCEPT': 4}),
             'COM_RCL_EXCEPT 不豁免 OFFBOARD') == 'fail')
check('MPC_XY_VEL_MAX=1.0 (<2) → fail',
      status(F.check_params({**good, 'MPC_XY_VEL_MAX': 1.0}),
             'MPC_XY_VEL_MAX ≥ 2.0（≥ 指令上限）') == 'fail')
check('RTL_RETURN_ALT>GF_MAX_VER_DIST → fail',
      status(F.check_params({**good, 'RTL_RETURN_ALT': 30.0, 'GF_MAX_VER_DIST': 15.0}),
             'RTL_RETURN_ALT ≤ GF_MAX_VER_DIST') == 'fail')
check('电池阈值次序错 → fail',
      status(F.check_params({**good, 'BAT1_LOW_THR': 0.05, 'BAT1_CRIT_THR': 0.10,
                             'BAT1_EMERGEN_THR': 0.15}),
             '电池阈值次序 EMERGEN ≤ CRIT ≤ LOW') == 'fail')

print('\n=== 实机（indoor）===')
indoor = {**good, 'GF_MAX_HOR_DIST': 3.0, 'GF_MAX_VER_DIST': 3.0,
          'RTL_RETURN_ALT': 2.0, 'COM_OBL_RC_ACT': 4, 'NAV_RCL_ACT': 3,
          'COM_LOW_BAT_ACT': 3, 'COM_OF_LOSS_T': 0.2,
          'EKF2_GPS_CTRL': 0, 'EKF2_EV_CTRL': 11, 'EKF2_BARO_CTRL': 0, 'EKF2_MAG_TYPE': 5,
          'SENS_BOARD_ROT': 0}
check('indoor 合规无 fail', not has_fail(F.check_params(indoor, 'indoor')))
check('围栏未启用 → fail',
      status(F.check_params({**indoor, 'GF_MAX_HOR_DIST': 0}, 'indoor'),
             '围栏已启用 (GF_MAX_* > 0)') == 'fail')
check('COM_OBL_RC_ACT=0(Position) → fail',
      status(F.check_params({**indoor, 'COM_OBL_RC_ACT': 0}, 'indoor'),
             'COM_OBL_RC_ACT = 4(Land)') == 'fail')
check('NAV_RCL_ACT=2(Return) → fail',
      status(F.check_params({**indoor, 'NAV_RCL_ACT': 2}, 'indoor'),
             'NAV_RCL_ACT = 3(Land)') == 'fail')
check('COM_LOW_BAT_ACT=0 → fail',
      status(F.check_params({**indoor, 'COM_LOW_BAT_ACT': 0}, 'indoor'),
             'COM_LOW_BAT_ACT ∈ {2,3}') == 'fail')
check('EKF2 源错（GPS 开着）→ fail',
      status(F.check_params({**indoor, 'EKF2_GPS_CTRL': 7}, 'indoor'),
             'EKF2 源=动捕(0/11/0/5)') == 'fail')

print('\n=== 实机（outdoor/RTK）===')
outdoor = {**good, 'GF_MAX_HOR_DIST': 12.0, 'GF_MAX_VER_DIST': 15.0, 'RTL_RETURN_ALT': 10.0,
           'COM_OBL_RC_ACT': 4, 'NAV_RCL_ACT': 3, 'COM_LOW_BAT_ACT': 3, 'COM_OF_LOSS_T': 0.2,
           'EKF2_GPS_CTRL': 7, 'EKF2_EV_CTRL': 0, 'EKF2_BARO_CTRL': 1, 'EKF2_MAG_TYPE': 0}
check('outdoor 合规无 fail', not has_fail(F.check_params(outdoor, 'outdoor')))
check('outdoor EKF2 源错 → fail',
      status(F.check_params({**outdoor, 'EKF2_EV_CTRL': 11}, 'outdoor'),
             'EKF2 源=RTK(7/0/1/0)') == 'fail')

print('\n=== PRESETS 自洽 ===')
check('预置参数名合法且为数值',
      all(isinstance(n, str) and n.isupper() and isinstance(v, (int, float))
          for grp in F.PRESETS.values() for (n, v, _) in grp))
check('failsafe 组含 Land 动作',
      any(n == 'COM_OBL_RC_ACT' and int(v) == 4 for (n, v, _) in F.PRESETS['failsafe'])
      and any(n == 'NAV_RCL_ACT' and int(v) == 3 for (n, v, _) in F.PRESETS['failsafe']))

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_fcu_params 全部通过')
