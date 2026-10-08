#!/usr/bin/env python3
"""contact_detect.py —— 接触检测（真机"接住"的物理触发），纯 Python、可离线自测。

动机：仿真里"接住"是软件判据（位置+速度），真机需要**真实接触事件**来触发锁扣/夹爪。
本模块融合三类信号：
  · external  ：外部力开关/微动/红外对射（可靠，有则优先）；
  · accel 尖峰：B 垂向加速度在载荷落上瞬间的冲击（由 PX4 az 或速度差分得到）；
  · 速度反转 ：载荷相对 B 的垂向闭合速度在接触后骤减/反向。

用法:
  python3 -m payload_catch.contact_detect     # 自测
"""
from __future__ import annotations


class ContactDetector:
    def __init__(self, accel_thresh: float = 15.0, v_approach: float = 0.5,
                 v_reversal: float = 0.2, hold_s: float = 0.15, use_accel: bool = True):
        self.accel_thresh = accel_thresh
        self.v_approach = v_approach
        self.v_reversal = v_reversal
        self.hold_s = hold_s
        self.use_accel = use_accel
        self._t_contact = None
        self._prev_rel_vz = None

    def update(self, t: float, az: float, rel_vz: float, near: bool,
               external: bool = False) -> bool:
        """返回"是否接触"（检测后保持 hold_s）。

        az      : B 的垂向加速度 (NED, m/s²)，下为正；
        rel_vz  : 载荷相对 B 的垂向速度 (NED, m/s)，`v_pay_z − v_B_z`，下为正；
        near    : 载荷是否在托盘口窗口内（由调用方判定）；
        external: 外部接触开关（力/微动/对射）。
        """
        hit = False
        if external:
            hit = True
        elif near:
            if self.use_accel and abs(az) >= self.accel_thresh:
                hit = True
            if (self._prev_rel_vz is not None and self._prev_rel_vz > self.v_approach
                    and rel_vz < self.v_reversal):
                hit = True                      # 闭合速度骤减/反转 → 接触
        self._prev_rel_vz = rel_vz
        if hit:
            self._t_contact = t
        if self._t_contact is not None and (t - self._t_contact) <= self.hold_s:
            return True
        return False

    def reset(self):
        self._t_contact = None
        self._prev_rel_vz = None


def _selftest():
    ok = True
    # 1) 加速度尖峰触发
    d = ContactDetector(accel_thresh=15.0)
    fired = False
    for i in range(10):
        t = i * 0.01
        r = d.update(t, az=(30.0 if i == 5 else 0.0), rel_vz=3.0, near=True)
        fired |= r
    ok &= fired
    print(f'[1] 加速度尖峰触发: {"OK" if fired else "FAIL"}')

    # 2) 速度反转触发（无加速度尖峰）
    d2 = ContactDetector(accel_thresh=100.0)   # 关掉加速度触发
    fired2 = False
    seq = [3.5, 3.0, 2.0, 0.1, 0.0]
    for i, rv in enumerate(seq):
        fired2 |= d2.update(i * 0.01, az=0.0, rel_vz=rv, near=True)
    ok &= fired2
    print(f'[2] 速度反转触发: {"OK" if fired2 else "FAIL"}')

    # 3) 外部开关
    d3 = ContactDetector()
    ok &= d3.update(0.0, 0.0, 0.0, near=False, external=True)
    print(f'[3] 外部开关: {"OK" if d3.update(0.0,0,0,False,external=True) else "FAIL"}')

    # 4) 保持 hold_s
    d4 = ContactDetector(accel_thresh=15.0, hold_s=0.2)
    d4.update(0.0, az=30.0, rel_vz=3.0, near=True)
    ok &= d4.update(0.1, az=0.0, rel_vz=0.0, near=True)
    ok &= not d4.update(0.5, az=0.0, rel_vz=0.0, near=True)
    print(f'[4] hold 保持/超时: {"OK" if ok else "FAIL"}')

    # 5) 远离时不应触发
    d5 = ContactDetector(accel_thresh=15.0)
    ok &= not d5.update(0.0, az=100.0, rel_vz=5.0, near=False)
    print(f'[5] 远离不触发: {"OK" if not d5.update(0.0,100,5,False) else "FAIL"}')

    print('contact_detect 自测', '通过 ✅' if ok else '失败 ❌')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
