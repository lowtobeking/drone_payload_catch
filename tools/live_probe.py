#!/usr/bin/env python3
"""运行时（起飞前）**持续**探测 PX4 —— ROS-native，替代 `ros2 topic echo --once` 快照。

参考 drone_package_20260908 的 `ekf_watch.py`（持续订阅 EKF）、`sensor_check.py`
（传感器/磁罗盘健康）、`gps_acceptance.py`（**持续** eph/epv/sats，快照会骗人）。
本工具订阅 `--seconds` 秒（默认 3），对**整段窗口**取聚合（比例/最坏值），
再交给**纯函数 `evaluate()`** 判定。

    python3 tools/live_probe.py                       # drone 0，3s
    python3 tools/live_probe.py --drone both --seconds 5
    python3 tools/live_probe.py --gps                 # 室外：加 GPS 严格门（eph≤1.5/epv≤2.5/sats≥20）
    python3 tools/live_probe.py --json                # 机器可读（供 preflight_check --live 合并）

无需 ROS 也能 import（rclpy 缺失时 HAVE_ROS=False，仅 `main` 拒绝运行）——
`tools/test_live_probe.py` 就是靠这一点离线单测 `evaluate()`。
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from px4_msgs.msg import (EstimatorStatusFlags, FailsafeFlags, SensorGps,
                              VehicleLocalPosition)
    HAVE_ROS = True
except Exception:                                             # noqa: BLE001
    HAVE_ROS = False

# 阈值（可被命令行覆盖）
EPH_MAX, EPV_MAX = 0.50, 0.50          # 与 px4_iface sensor_eph/epv_max 默认一致
GPS_EPH_MAX, GPS_EPV_MAX, GPS_SATS_MIN = 1.5, 2.5, 20   # 参考 gps_acceptance

# 注意：PX4-1.16 的 uXRCE-DDS **不桥接** vehicle_imu_status / vehicle_magnetometer，
# IMU/磁健康改走已桥接的 estimator_status_flags（fs_bad_acc_* / cs_mag* / fs_bad_mag_*）；
# GPS 话题为 /fmu/out/vehicle_gps_position（类型 SensorGps），不是 sensor_gps。

FAILSAFE_FATAL = [
    ('local_position_invalid', 'failsafe: 本地位置无效'),
    ('local_velocity_invalid', 'failsafe: 本地速度无效'),
    ('attitude_invalid', 'failsafe: 姿态无效'),
    ('geofence_breached', 'failsafe: 越界'),
    ('fd_critical_failure', 'failsafe: 临界故障'),
]


@dataclass
class Snapshot:
    """一段窗口的聚合量（纯数据，便于离线构造与单测）。"""
    n_lpos: int = 0
    xy_valid_frac: float = 0.0
    z_valid_frac: float = 0.0
    v_valid_frac: float = 0.0
    dead_reckoning_frac: float = 0.0
    heading_good_frac: float = 0.0
    max_eph: float = 0.0
    max_epv: float = 0.0
    n_failsafe: int = 0
    failsafe: dict = field(default_factory=dict)   # 窗口内**曾**为 true 的坏位
    n_est: int = 0
    est: dict = field(default_factory=dict)        # 窗口内**曾**为 true 的位
    n_gps: int = 0
    gps_max_eph: float = 0.0
    gps_max_epv: float = 0.0
    gps_min_sats: int = 999


def evaluate(snap: Snapshot, *, require_gps: bool = False, eph_max: float = EPH_MAX,
             epv_max: float = EPV_MAX, gps_eph_max: float = GPS_EPH_MAX,
             gps_epv_max: float = GPS_EPV_MAX,
             gps_sats_min: int = GPS_SATS_MIN) -> list:
    """纯函数：把窗口聚合量判成 ✅/⚠️/❌ 列表。"""
    R: list = []

    def add(name, status, detail=''):
        R.append({'name': name, 'status': status, 'detail': detail})

    # ── 话题 / EKF 有效性 ──
    if snap.n_lpos == 0:
        add('vehicle_local_position', 'fail', '未收到（PX4/agent 未起？）')
    else:
        add('lpos 发布', 'ok', f'{snap.n_lpos} 帧/{_pct(snap.xy_valid_frac)}')
        add('xy_valid 持续', 'ok' if snap.xy_valid_frac > 0.95 else 'fail',
            _pct(snap.xy_valid_frac))
        add('z_valid 持续', 'ok' if snap.z_valid_frac > 0.95 else 'fail',
            _pct(snap.z_valid_frac))
        add('v_xy/z_valid 持续', 'ok' if snap.v_valid_frac > 0.95 else 'fail',
            _pct(snap.v_valid_frac))
        add('非 dead_reckoning', 'ok' if snap.dead_reckoning_frac < 0.05 else 'fail',
            _pct(snap.dead_reckoning_frac))
        add('heading_good_for_control',
            'ok' if snap.heading_good_frac > 0.5 else 'warn',
            _pct(snap.heading_good_frac) + '（本仿真常 false）')
        add(f'eph ≤ {eph_max}', 'ok' if snap.max_eph <= eph_max else 'fail',
            f'{snap.max_eph:.3f}')
        add(f'epv ≤ {epv_max}', 'ok' if snap.max_epv <= epv_max else 'fail',
            f'{snap.max_epv:.3f}')

    # ── failsafe flags ──
    if snap.n_failsafe == 0:
        add('failsafe_flags', 'warn', '未收到')
    else:
        for flag, label in FAILSAFE_FATAL:
            bad = bool(snap.failsafe.get(flag))
            add(label, 'fail' if bad else 'ok')
        # offboard 信号：**起飞前尚未进 offboard 时 true 属正常**，不能当硬失败；
        # 主动验证（发心跳看翻转）由 tools/uplink_test.py 负责。
        off = bool(snap.failsafe.get('offboard_control_signal_lost'))
        add('failsafe: offboard 信号', 'warn' if off else 'ok',
            '起 offboard 前 true 正常（用 uplink_test 主动验证）' if off else '')
        add('battery_unhealthy', 'warn' if snap.failsafe.get('battery_unhealthy') else 'ok',
            '已知 B 固有告警' if snap.failsafe.get('battery_unhealthy') else '')

    # ── estimator status（磁罗盘在线/故障）──
    if snap.n_est == 0:
        add('estimator_status_flags', 'warn', '未收到')
    else:
        add('tilt/yaw 对齐',
            'ok' if (snap.est.get('cs_tilt_align') and snap.est.get('cs_yaw_align')) else 'fail')
        mag_online = any(snap.est.get(k) for k in ('cs_mag', 'cs_mag_hdg', 'cs_mag_3d'))
        add('磁罗盘在线', 'ok' if mag_online else 'warn',
            '' if mag_online else '未融合磁（vision/无磁场景可接受）')
        add('无磁故障', 'fail' if snap.est.get('cs_mag_fault') else 'ok')
        add('磁未受扰', 'warn' if snap.est.get('cs_mag_field_disturbed') else 'ok')
        bad_mag = any(snap.est.get(k) for k in
                      ('fs_bad_mag_x', 'fs_bad_mag_y', 'fs_bad_mag_z', 'fs_bad_hdg'))
        add('无磁融合数值错误', 'fail' if bad_mag else 'ok')
        bad_acc = any(snap.est.get(k) for k in
                      ('fs_bad_acc_vertical', 'fs_bad_acc_clipping'))
        add('无加计故障(垂直/削波)', 'fail' if bad_acc else 'ok')

    # ── GPS（室外；默认只在 --gps 时判）──
    if require_gps:
        if snap.n_gps == 0:
            add('vehicle_gps_position', 'fail', '未收到（室外需要）')
        else:
            add(f'GPS eph ≤ {gps_eph_max}',
                'ok' if snap.gps_max_eph <= gps_eph_max else 'fail',
                f'{snap.gps_max_eph:.2f}')
            add(f'GPS epv ≤ {gps_epv_max}',
                'ok' if snap.gps_max_epv <= gps_epv_max else 'fail',
                f'{snap.gps_max_epv:.2f}')
            add(f'GPS sats ≥ {gps_sats_min}',
                'ok' if snap.gps_min_sats >= gps_sats_min else 'fail',
                str(snap.gps_min_sats))
    return R


def _pct(x: float) -> str:
    return f'{100.0 * x:.0f}%'


# ------------------------------------------------------------------ 采集
if HAVE_ROS:
    class _Acc:
        def __init__(self):
            self.s = Snapshot()
            self._n = 0
            self._sum = dict(xy=0, z=0, v=0, dr=0, hdg=0)

        def lpos(self, m):
            self._n += 1
            self._sum['xy'] += bool(m.xy_valid)
            self._sum['z'] += bool(m.z_valid)
            self._sum['v'] += bool(m.v_xy_valid and m.v_z_valid)
            self._sum['dr'] += bool(m.dead_reckoning)
            self._sum['hdg'] += bool(m.heading_good_for_control)
            self.s.max_eph = max(self.s.max_eph, float(m.eph))
            self.s.max_epv = max(self.s.max_epv, float(m.epv))
            self._finalize_lpos()

        def _finalize_lpos(self):
            n = max(self._n, 1)
            self.s.n_lpos = self._n
            self.s.xy_valid_frac = self._sum['xy'] / n
            self.s.z_valid_frac = self._sum['z'] / n
            self.s.v_valid_frac = self._sum['v'] / n
            self.s.dead_reckoning_frac = self._sum['dr'] / n
            self.s.heading_good_frac = self._sum['hdg'] / n

        def failsafe(self, m):
            self.s.n_failsafe += 1
            for f, _ in FAILSAFE_FATAL:
                self.s.failsafe[f] = self.s.failsafe.get(f) or bool(getattr(m, f, False))
            self.s.failsafe['offboard_control_signal_lost'] = (
                self.s.failsafe.get('offboard_control_signal_lost')
                or bool(m.offboard_control_signal_lost))
            self.s.failsafe['battery_unhealthy'] = (
                self.s.failsafe.get('battery_unhealthy') or bool(m.battery_unhealthy))

        def est(self, m):
            self.s.n_est += 1
            for k in ('cs_tilt_align', 'cs_yaw_align', 'cs_mag', 'cs_mag_hdg', 'cs_mag_3d',
                      'cs_mag_fault', 'cs_mag_field_disturbed', 'fs_bad_mag_x',
                      'fs_bad_mag_y', 'fs_bad_mag_z', 'fs_bad_hdg',
                      'fs_bad_acc_vertical', 'fs_bad_acc_clipping'):
                self.s.est[k] = self.s.est.get(k) or bool(getattr(m, k, False))

        def gps(self, m):
            self.s.n_gps += 1
            self.s.gps_max_eph = max(self.s.gps_max_eph, float(m.eph))
            self.s.gps_max_epv = max(self.s.gps_max_epv, float(m.epv))
            self.s.gps_min_sats = min(self.s.gps_min_sats, int(m.satellites_used))

    def _qos():
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class _Probe(Node):
        def __init__(self, drone_ids):
            super().__init__('live_probe')
            self.acc = {i: _Acc() for i in drone_ids}
            q = _qos()
            for i in drone_ids:
                p = '/fmu' if i == 0 else f'/px4_{i}/fmu'
                a = self.acc[i]
                self.create_subscription(VehicleLocalPosition, f'{p}/out/vehicle_local_position', a.lpos, q)
                self.create_subscription(FailsafeFlags, f'{p}/out/failsafe_flags', a.failsafe, q)
                self.create_subscription(EstimatorStatusFlags, f'{p}/out/estimator_status_flags', a.est, q)
                # PX4-1.16 桥接的是 vehicle_gps_position（类型 SensorGps）
                self.create_subscription(SensorGps, f'{p}/out/vehicle_gps_position', a.gps, q)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='运行时 PX4 持续探测（起飞前）')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--drone', default='both', choices=['0', '1', 'both'])
    ap.add_argument('--gps', action='store_true', help='加 GPS 严格门（室外）')
    ap.add_argument('--gps-eph-max', type=float, default=GPS_EPH_MAX)
    ap.add_argument('--gps-epv-max', type=float, default=GPS_EPV_MAX)
    ap.add_argument('--gps-sats-min', type=int, default=GPS_SATS_MIN)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    if not HAVE_ROS:
        print('❌ live_probe 需要 ROS（rclpy/px4_msgs）；请先 source env.sh')
        return 2

    ids = [0, 1] if args.drone == 'both' else [int(args.drone)]
    rclpy.init()
    node = _Probe(ids)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    out = {}
    n_fail = 0
    for i in ids:
        res = evaluate(node.acc[i].s, require_gps=args.gps,
                       gps_eph_max=args.gps_eph_max, gps_epv_max=args.gps_epv_max,
                       gps_sats_min=args.gps_sats_min)
        out[i] = res
        n_fail += sum(r['status'] == 'fail' for r in res)
    node.destroy_node()
    rclpy.shutdown()

    if args.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        icon = {'ok': '✅', 'warn': '⚠️ ', 'fail': '❌'}
        for i in ids:
            print(f'--- drone {i} ---')
            for r in out[i]:
                print(f'  {icon[r["status"]]} {r["name"]}' + (f'   — {r["detail"]}' if r['detail'] else ''))
        print(f'--- 汇总：fail={n_fail} ---')
    return 1 if n_fail else 0


if __name__ == '__main__':
    raise SystemExit(main())
