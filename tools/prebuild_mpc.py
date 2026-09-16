#!/usr/bin/env python3
"""预热编译 b_node 用的 acados 终端 MPC，避免 SITL 启动期编译尖峰。

用法: source ~/drone_payload_catch/env.sh && python3 ~/drone_payload_catch/tools/prebuild_mpc.py
命中缓存时 ~0.1s；首次生成+编译数秒。参数必须与 launch 里 b_node 的 MPC 一致。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.mpc_terminal import TerminalMPC          # noqa: E402


def main():
    mpc = TerminalMPC(N=30, dt=0.02, a_max=10.0, v_max=9.0)
    ref_t = np.linspace(0.0, 0.5, 26)
    ref_p = np.zeros((26, 3)); ref_v = np.zeros((26, 3))
    ref_p[:, 2] = -np.linspace(0.0, 0.5, 26)
    ref_v[:, 2] = 1.0
    mpc.solve(np.zeros(6), ref_t, ref_p, ref_v,
              np.array([0.0, 0.0, -0.5]), np.array([0.0, 0.0, 1.0]))
    print('[prebuild] MPC OK')


if __name__ == '__main__':
    main()
