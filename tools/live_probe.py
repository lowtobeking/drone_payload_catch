#!/usr/bin/env python3
"""Both-aircraft sustained preflight health probe; fail closed, battery >=40%."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_catch.preflight_health import (Snapshot, evaluate, HealthAccumulator,
    EPH_MAX, EPV_MAX, GPS_EPH_MAX, GPS_EPV_MAX, GPS_SATS_MIN, FAILSAFE_FATAL,
    WINDOW_S)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (BatteryStatus, EstimatorStatusFlags, FailsafeFlags,
                             SensorGps, VehicleLocalPosition, VehicleAttitude, VehicleStatus, ManualControlSetpoint)
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

if HAVE_ROS:
    class _Probe(Node):
        def __init__(self, ids):
            super().__init__('live_probe')
            self.acc = {i: HealthAccumulator() for i in ids}
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=5,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
            streams = [(VehicleLocalPosition, 'lpos', 'vehicle_local_position'),
                       (VehicleAttitude, 'att', 'vehicle_attitude'),
                       (VehicleStatus, 'status', 'vehicle_status_v1'),
                       (FailsafeFlags, 'failsafe', 'failsafe_flags'),
                       (EstimatorStatusFlags, 'est', 'estimator_status_flags'),
                       (BatteryStatus, 'batt', 'battery_status'),
                       (SensorGps, 'gps', 'vehicle_gps_position'),
                       (ManualControlSetpoint,'rc','manual_control_setpoint')]
            for i in ids:
                prefix = '/fmu' if i == 0 else f'/px4_{i}/fmu'
                for msg_type, key, suffix in streams:
                    self.create_subscription(msg_type, f'{prefix}/out/{suffix}',
                        lambda m, i=i, key=key: self.acc[i].record(key, m, time.monotonic()), q)


def main(argv=None):
    ap = argparse.ArgumentParser(description='强制关键健康数据／40% 电池起飞检查')
    ap.add_argument('--seconds', type=float, default=WINDOW_S)
    ap.add_argument('--drone', default='both', choices=['0', '1', 'both'])
    ap.add_argument('--gps', action='store_true')
    ap.add_argument('--rc',action='store_true',help='require independent radio receiver input')
    ap.add_argument('--gps-eph-max', type=float, default=GPS_EPH_MAX)
    ap.add_argument('--gps-epv-max', type=float, default=GPS_EPV_MAX)
    ap.add_argument('--gps-sats-min', type=int, default=GPS_SATS_MIN)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)
    if not HAVE_ROS:
        print('live_probe 需要 ROS（rclpy/px4_msgs）；拒绝降级放行', file=sys.stderr)
        return 2
    if not WINDOW_S <= args.seconds <= 30:
        ap.error('--seconds 必须在 3–30 秒之间')
    ids = [0, 1] if args.drone == 'both' else [int(args.drone)]
    rclpy.init()
    node = _Probe(ids)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        out = {i: evaluate(acc.snapshot(time.monotonic()), require_gps=args.gps,require_rc=args.rc,
                          gps_eph_max=args.gps_eph_max, gps_epv_max=args.gps_epv_max,
                          gps_sats_min=args.gps_sats_min) for i, acc in node.acc.items()}
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        for i, results in out.items():
            print(f'--- drone {i} ---')
            for r in results:
                print(f'{r["status"]}: {r["name"]} {r["detail"]}')
    return int(any(r['status'] == 'fail' for results in out.values() for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
