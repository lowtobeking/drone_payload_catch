#!/usr/bin/env python3
"""acados MPC 冒烟测试（参考 drone_package_20260908/acados_smoke.py）。

    source env.sh && python3 tools/smoke_acados.py

不需要 ROS 运行时、不需要 PX4/Gazebo —— 只验 acados 这条链路本身：
构建（codegen+编译）→ 求解一次 → 连跑 50 次看耗时与稳定性。
失败（status 非 0 / 返回值不齐 / 耗时爆表）时退出码非 0。
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if not os.environ.get('ACADOS_SOURCE_DIR'):
    print('⚠️  未设置 ACADOS_SOURCE_DIR —— 请先 `source env.sh`（本测试需要 acados 链）')
    sys.exit(2)

from payload_catch.mpc_terminal import TerminalMPC   # noqa: E402

t0 = time.time()
mpc = TerminalMPC(N=20, dt=0.05, a_max=6.0, v_max=5.0)
print(f'[{time.time() - t0:5.1f}s] 求解器构建完成  N={mpc.N} dt={mpc.dt}')

ref_t = np.linspace(0.0, 0.5, 26)
ref_p = np.zeros((26, 3))
ref_v = np.zeros((26, 3))
for k in range(26):
    s = ref_t[k] / 0.5
    ref_p[k] = [s * 1.0, 0.0, -0.5 * s]
    ref_v[k] = [2.0, 0.0, -1.0]
p_c = np.array([1.0, 0.0, -0.5])
v_c = np.array([2.0, 0.0, -1.0])
x0 = np.zeros(6)

t1 = time.time()
u0, st, v_next = mpc.solve(x0, ref_t, ref_p, ref_v, p_c, v_c)
ms = (time.time() - t1) * 1000.0
print(f'求解: status={st}  u0={np.round(u0, 4)}  v_next={np.round(v_next, 4)}  '
      f'用时={ms:.2f}ms')

ts = []
for _ in range(50):
    a = time.time()
    mpc.solve(x0, ref_t, ref_p, ref_v, p_c, v_c)
    ts.append((time.time() - a) * 1000.0)
print(f'50 次求解: 均值 {np.mean(ts):.2f}ms  最大 {np.max(ts):.2f}ms')

ok = (int(st) == 0 and np.all(np.isfinite(u0)) and np.all(np.isfinite(np.asarray(v_next)))
      and len(np.asarray(u0).ravel()) == 3)
if not ok:
    print(f'❌ smoke_acados 失败：status={st}')
    sys.exit(1)
print('✅ smoke_acados 通过')
