#!/usr/bin/env python3
"""上行链路判定逻辑（uplink_test.verdict）离线单测——无需 ROS。

    python3 tools/test_uplink_test.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.uplink_test import verdict   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


ok, d = verdict([True, True, False, False])       # 曾收到流
check('出现过 False → 通过', ok)
ok, d = verdict([True, True, True])               # 一直 lost
check('一直 lost → 失败', not ok, d)
ok, d = verdict([])                               # 没收到
check('无采样 → 失败', not ok, d)
ok, d = verdict([False])
check('单次 False → 通过', ok)

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_uplink_test 全部通过')
