#!/usr/bin/env python3
"""遥测有效性判据（payload_catch/telemetry.py）离线单测——合成日志，无需 ROS。

    python3 tools/test_telemetry.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import telemetry as T   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def line(src, t, pos, safe='OK'):
    return (f'[{src}-1] [INFO] [{t:.2f}] [{src}]: {src} phase=DIVE safe={safe} '
            f'pos_w=[{pos[0]} {pos[1]} {pos[2]}] vel=[0.1 0.2 0.3] caught=False')


def codes(issues):
    return sorted(i.code for i in issues)


print('=== longest_const_run / max_t_gap ===')
check('全同 [1,1,1] = 3', T.longest_const_run([1, 1, 1]) == 3)
check('交替 = 1', T.longest_const_run([1, 2, 1, 2]) == 1)
check('空 = 0', T.longest_const_run([]) == 0)
check('max_t_gap', abs(T.max_t_gap([0.0, 1.0, 3.0, 3.5]) - 2.0) < 1e-9)
check('median_t_gap', abs(T.median_t_gap([0.0, 1.0, 2.0, 3.0]) - 1.0) < 1e-9)

print('\n=== parse_log ===')
txt = '\n'.join([line('a_node', i * 0.5, (i * 0.1, 0, -3)) for i in range(5)]
                + [line('b_node', i * 0.5, (0, i * 0.1, -3)) for i in range(4)])
series = T.parse_log(txt)
check('两个源', set(series) == {'a_node', 'b_node'}, str(list(series)))
check('a_node 5 帧', len(series['a_node']) == 5)
check('pos_w 解析正确', series['a_node'][1].pos == (0.1, 0.0, -3.0))

print('\n=== analyze：健康 → 无 fail ===')
txt = '\n'.join([line('b_node', i * 0.5, (0.1 * i, 0.2 * i, -3.0 - 0.1 * i))
                 for i in range(15)])
issues = T.analyze(T.parse_log(txt))
check('健康日志无 fail', all(i.level != 'fail' for i in issues), str(codes(issues)))

print('\n=== analyze：冻结（STALE）===')
txt = '\n'.join([line('b_node', i * 0.5, (1.0, 2.0, -3.0)) for i in range(15)])
issues = T.analyze(T.parse_log(txt))
check('STALE fail', any(i.code == 'STALE' and i.level == 'fail' for i in issues),
      str(codes(issues)))
check('冻结 15 帧', any(i.code == 'STALE' and '15/15' in i.detail for i in issues))

print('\n=== analyze：溢出（OVERFLOW）===')
txt = ('\n'.join([line('b_node', i * 0.5, (0.1 * i, 0, -3.0)) for i in range(5)])
       + '\n' + line('b_node', 3.0, (2.1e9, 0, -3.0)))
issues = T.analyze(T.parse_log(txt))
check('OVERFLOW fail', any(i.code == 'OVERFLOW' for i in issues), str(codes(issues)))

print('\n=== analyze：沉默（SILENCE，仅周期源）===')
# 周期源：12 拍 @0.5s 后出现大间隔 → SILENCE
times = [i * 0.5 for i in range(12)] + [30.0]
txt = '\n'.join([line('b_node', t, (t, 0, -3.0)) for t in times])
issues = T.analyze(T.parse_log(txt), max_gap=5.0)
check('周期源出现大间隔 → SILENCE', any(i.code == 'SILENCE' for i in issues), str(codes(issues)))
# 事件型源：常态间隔就很大 → 不误报
txt = '\n'.join([line('payload_node', i * 8.0, (i * 0.1, 0, -3.0)) for i in range(12)])
issues = T.analyze(T.parse_log(txt), max_gap=5.0)
check('事件型源长间隔 → 不误报 SILENCE',
      not any(i.code == 'SILENCE' for i in issues), str(codes(issues)))
# 样本太少 → 不判沉默（避免启动突刺误报）
txt = '\n'.join([line('payload_node', t, (t, 0, -3.0)) for t in (0.0, 10.0, 25.0)])
check('样本 <min_run → 不判沉默',
      not any(i.code == 'SILENCE' for i in T.analyze(T.parse_log(txt), max_gap=5.0)))

print('\n=== analyze：兜底（HOVER，仅 warn）===')
txt = '\n'.join([line('b_node', i * 0.5, (0.1 * i, 0, -3.0), safe='HOLD')
                 for i in range(14)]
                + [line('b_node', 7.0, (0.1 * 14, 0, -3.0), safe='OK')])
issues = T.analyze(T.parse_log(txt))
hov = [i for i in issues if i.code == 'HOVER']
check('HOVER warn', hov and hov[0].level == 'warn', str(codes(issues)))
check('HOVER 不导致 fail', all(i.level != 'fail' for i in issues))

print('\n=== validity_report 端到端 ===')
issues = T.validity_report(txt)
check('validity_report 可用', isinstance(issues, list) and len(issues) >= 1)

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_telemetry 全部通过')
