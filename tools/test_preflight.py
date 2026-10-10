#!/usr/bin/env python3
"""起飞前自检逻辑（preflight_check.py）离线单测——用假 env / 假文件系统覆盖缺项组合。

    python3 tools/test_preflight.py
"""
import io
import json
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.preflight_check import (LIVE_TOPICS, Res, _print,   # noqa: E402
                                   check_environment, check_px4_logs,
                                   live_checks, probe_live, probe_params)

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


GOOD_ENV = {
    'PX4_DIR': '/opt/px4',
    'SITL_WORLD': '/opt/px4/world.sdf',
    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
    'GZ_SIM_RESOURCE_PATH': '/opt/px4/models',
    'ACADOS_SOURCE_DIR': '/opt/acados',
}


def always_ok_env():
    return dict(GOOD_ENV)


def all_true(*_a, **_k):
    return True


def all_present(_m):
    return True


def which_ok(_b):
    return '/usr/bin/' + _b


def statuses(results, name):
    for r in results:
        if r.name == name:
            return r.status
    return None


print('=== 环境齐全 → 无 fail ===')
r = check_environment(always_ok_env(), '/repo', which=which_ok,
                      isfile=all_true, isdir=all_true, has_dep=all_present)
check('无 fail', all(x.status != 'fail' for x in r), f'{len(r)} 项')

print('\n=== 离线模式：ROS/PX4 缺失只 warn ===')
env = dict(GOOD_ENV)
env.pop('PX4_DIR')
env.pop('SITL_WORLD')
env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
r = check_environment(env, '/repo', strict_sitl=False, which=lambda b: None,
                      isfile=all_true, isdir=all_true,
                      has_dep=lambda m: m not in ('rclpy', 'px4_msgs'))
check('PX4_DIR 缺失 → warn', statuses(r, 'PX4_DIR 已设置') == 'warn')
check('SITL_WORLD 缺失 → warn', statuses(r, 'SITL_WORLD 存在') == 'warn')
check('RMW 不对 → warn', statuses(r, 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp') == 'warn')
check('rclpy 缺失 → warn', statuses(r, 'python:rclpy') == 'warn')
check('gz 缺失 → warn', statuses(r, 'binary:gz') == 'warn')
check('离线模式无 fail', all(x.status != 'fail' for x in r))

print('\n=== SITL 严格模式：同样缺失 → fail ===')
r = check_environment(env, '/repo', strict_sitl=True, which=lambda b: None,
                      isfile=all_true, isdir=all_true,
                      has_dep=lambda m: m not in ('rclpy', 'px4_msgs'))
check('PX4_DIR 缺失 → fail', statuses(r, 'PX4_DIR 已设置') == 'fail')
check('RMW 不对 → fail', statuses(r, 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp') == 'fail')
check('rclpy 缺失 → fail', statuses(r, 'python:rclpy') == 'fail')
check('gz 缺失 → fail', statuses(r, 'binary:gz') == 'fail')

print('\n=== 硬性缺项（任何模式都 fail）===')
r = check_environment(always_ok_env(), '/repo', which=which_ok,
                      isfile=lambda p: 'catch_scenarios' not in p,   # 配置缺失
                      isdir=all_true, has_dep=all_present)
check('配置缺失 → fail', statuses(r, 'config/catch_scenarios.yaml') == 'fail')
r = check_environment(always_ok_env(), '/repo', which=which_ok,
                      isfile=all_true, isdir=all_true,
                      has_dep=lambda m: m not in ('numpy',))
check('numpy 缺失 → fail', statuses(r, 'python:numpy') == 'fail')

print('\n=== acados 缺失始终只 warn ===')
r = check_environment(always_ok_env(), '/repo', which=which_ok,
                      isfile=all_true, isdir=lambda p: 'acados' not in p,
                      has_dep=lambda m: m != 'acados_template')
check('ACADOS_SOURCE_DIR 缺失 → warn', statuses(r, 'ACADOS_SOURCE_DIR 存在') == 'warn')
check('acados_template 缺失 → warn', statuses(r, 'python:acados_template') == 'warn')

print('\n=== _print 汇总计数 ===')
buf = io.StringIO()
with redirect_stdout(buf):
    n_fail, n_warn = _print([Res('a', 'ok'), Res('b', 'warn', 'w'),
                             Res('c', 'fail', 'f')])
check('fail 计数=1', n_fail == 1)
check('warn 计数=1', n_warn == 1)

print('\n=== live_checks（假 ros2 探测）===')


def fake_run_ok(cmd, timeout=8):
    if cmd[:3] == ['ros2', 'topic', 'list']:
        return 0, '\n'.join(LIVE_TOPICS)
    if cmd[3:4] == ['/fmu/out/failsafe_flags']:
        return 0, 'local_position_invalid: false\n'
    return 0, 'xy_valid: true\nz_valid: true\ndead_reckoning: false\n'


r = live_checks(runner=fake_run_ok)
check('live 全 OK', all(x.status == 'ok' for x in r), f'{len(r)} 项')


def fake_run_bad(cmd, timeout=8):
    if cmd[:3] == ['ros2', 'topic', 'list']:
        return 0, '/fmu/out/vehicle_status_v1'
    return 0, ''


r = live_checks(runner=fake_run_bad)
check('缺话题 → fail', any(x.status == 'fail' for x in r))
check('EKF 无效 → fail', statuses(r, 'EKF xy_valid') == 'fail')

print('\n=== check_px4_logs（起飞前日志门：致命 vs 告警）===')
ok_logs = {'/logs/px4_0.log': 'boot\nReady for takeoff\n',
           '/logs/px4_1.log': 'Ready for takeoff\n'}
r = check_px4_logs('/logs', isfile=lambda p: p in ok_logs, read_text=lambda p: ok_logs[p])
check('日志正常 → 无 fail', all(x.status != 'fail' for x in r))

# 瞬态/已知告警：Preflight Fail 但已 Ready → 只 warn，不算 fail
transient = {'/logs/px4_0.log': 'Preflight Fail: ekf2 missing data\nReady for takeoff\n',
             '/logs/px4_1.log': 'Ready for takeoff\n'}
r = check_px4_logs('/logs', isfile=lambda p: p in transient, read_text=lambda p: transient[p])
check('瞬态 Preflight Fail → warn 非 fail',
      statuses(r, 'd0 Preflight 告警') == 'warn' and all(x.status != 'fail' for x in r))
check('瞬态时无致命故障 → ok', statuses(r, 'd0 无致命故障') == 'ok')

# 致命：Gyro STALE
fatal = {'/logs/px4_0.log': 'Ready for takeoff\nGyro #0 fail: STALE\n',
         '/logs/px4_1.log': 'Ready for takeoff\n'}
r = check_px4_logs('/logs', isfile=lambda p: p in fatal, read_text=lambda p: fatal[p])
check('Gyro STALE → fail', statuses(r, 'd0 无致命故障') == 'fail')

# 未就绪
nordy = {'/logs/px4_0.log': 'boot\n', '/logs/px4_1.log': 'Ready for takeoff\n'}
r = check_px4_logs('/logs', isfile=lambda p: p in nordy, read_text=lambda p: nordy[p])
check('未 Ready → fail', statuses(r, 'd0 Ready for takeoff') == 'fail')

r = check_px4_logs('/logs', isfile=lambda p: False, read_text=lambda p: '')
check('日志缺失 → warn', all(x.status == 'warn' for x in r))

print('\n=== probe_live（JSON 合并 / 无 ROS 回退）===')
payload = json.dumps({'0': [{'name': 'lpos 发布', 'status': 'ok', 'detail': '3 帧'}],
                      '1': [{'name': 'IMU 在线(device_id)', 'status': 'fail', 'detail': ''}]})
r = probe_live(runner=lambda cmd, timeout=8: (1, payload))
check('解析 JSON → 合并为 d0/d1', r is not None and len(r) == 2
      and r[0].name == 'd0: lpos 发布' and r[1].status == 'fail')
r = probe_live(runner=lambda cmd, timeout=8: (2, '❌ live_probe 需要 ROS'))
check('无 ROS（非 JSON）→ None 回退', r is None)

print('\n=== probe_params ===')
check('无来源 → None', probe_params() is None)
pp = json.dumps({'issues': [{'name': 'MPC_THR_MIN < MPC_THR_HOVER', 'status': 'ok',
                             'detail': ''},
                            {'name': 'COM_RCL_EXCEPT 不豁免 OFFBOARD', 'status': 'fail',
                             'detail': '=4'}]})
r = probe_params(runner=lambda cmd, timeout=8: (1, pp), param_file='/tmp/p.txt')
check('合并参数检查结果', r is not None and len(r) == 2
      and r[0].name == 'fcu: MPC_THR_MIN < MPC_THR_HOVER' and r[1].status == 'fail')

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_preflight 全部通过')
