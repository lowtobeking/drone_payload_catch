#!/usr/bin/env python3
"""Ground-only, single-restart A mocap recovery with a logged health window."""
import fcntl
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (EstimatorStatusFlags, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from std_msgs.msg import String

from payload_catch.px4_iface import qos_in

BASE = Path('/home/nvidia/payload_catch_deploy')
SERVICE = 'payload-catch-mocap-a.service'


def main():
    run_dir = BASE / 'flight_logs' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')
                                    + 'mocap_recovery_' + uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True)
    print('Run: ' + str(run_dir), flush=True)
    latest = {}
    outcome = {'passed': False, 'reason': 'recovery not started'}
    lock = (BASE / 'mission.lock').open('a')
    node = None
    with (run_dir / 'events.jsonl').open('w', encoding='utf-8', buffering=1) as events:
        def record(kind, data):
            events.write(json.dumps({'wall_s': time.time(), 'kind': kind, 'data': data},
                                    allow_nan=False, separators=(',', ':')) + '\n')

        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            rclpy.init(args=[])
            node = Node('a_mocap_recovery_readonly')

            def receive(key, msg):
                now = time.monotonic()
                if key == 'bridge':
                    data = json.loads(msg.data)
                    data = {field: data.get(field) for field in
                            ('forwarded', 'stale', 'latched_fault', 'last_error')}
                elif key == 'status':
                    data = {'arming_state': msg.arming_state,
                            'system_id': msg.system_id, 'component_id': msg.component_id}
                elif key == 'land':
                    data = {'landed': msg.landed, 'at_rest': msg.at_rest}
                elif key == 'flags':
                    data = {field: getattr(msg, field) for field in
                            ('cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt')}
                else:
                    data = {field: getattr(msg, field) for field in
                            ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid',
                             'heading_good_for_control', 'dead_reckoning')}
                latest[key] = (now, data)
                if key != 'local':
                    record(key, data)

            for key, cls, topic in (
                ('status', VehicleStatus, '/fmu/out/vehicle_status_v1'),
                ('land', VehicleLandDetected, '/fmu/out/vehicle_land_detected'),
                ('flags', EstimatorStatusFlags, '/fmu/out/estimator_status_flags'),
                ('local', VehicleLocalPosition, '/fmu/out/vehicle_local_position')):
                node.create_subscription(cls, topic,
                    lambda msg, key=key: receive(key, msg), qos_in())
            node.create_subscription(String, '/payload/mocap/a/bridge_status',
                lambda msg: receive('bridge', msg), 5)

            def fresh(key):
                return key in latest and time.monotonic() - latest[key][0] < 1.5

            def grounded():
                return (fresh('status') and fresh('land')
                    and latest['status'][1]['system_id'] == 1
                    and latest['status'][1]['component_id'] == 1
                    and latest['status'][1]['arming_state'] == 1
                    and latest['land'][1]['landed'] is True)

            ground_since = None
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if grounded():
                    if ground_since is None:
                        ground_since = time.monotonic()
                    if time.monotonic() - ground_since >= 2:
                        break
                else:
                    ground_since = None
            else:
                raise RuntimeError('A not continuously landed and disarmed; bridge not restarted')

            record('action', {'service_restart': SERVICE})
            with (run_dir / 'restart.log').open('w', encoding='utf-8') as log:
                proc = subprocess.run(['sudo', '-n', 'systemctl', 'restart', SERVICE],
                    cwd=BASE, timeout=30, stdout=log, stderr=subprocess.STDOUT, shell=False)
            if proc.returncode:
                raise RuntimeError('bridge restart failed; see restart.log')

            latest.pop('bridge', None)
            healthy_since = None
            forwarded = None
            last_advance = None
            reason = 'waiting for bridge and PX4 fusion'
            restarted_at = time.monotonic()
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if fresh('status') and latest['status'][1]['arming_state'] != 1:
                    raise RuntimeError('A armed during recovery')
                if fresh('land') and latest['land'][1]['landed'] is not True:
                    raise RuntimeError('A left the ground during recovery')
                if not grounded():
                    healthy_since = None
                    reason = 'landed/disarmed telemetry missing'
                    if time.monotonic() - restarted_at > 3:
                        raise RuntimeError(reason)
                    continue
                if not fresh('bridge'):
                    healthy_since = None
                    reason = 'bridge status missing/stale'
                    continue
                bridge = latest['bridge'][1]
                if bridge['latched_fault']:
                    raise RuntimeError('bridge latched: ' + bridge['latched_fault'])
                count = bridge['forwarded']
                if type(count) is not int or bridge['stale'] is not False:
                    healthy_since = None
                    reason = 'bridge stale or forwarding count invalid'
                    continue
                if forwarded is None:
                    forwarded = count
                    last_advance = time.monotonic()
                elif count > forwarded:
                    forwarded = count
                    last_advance = time.monotonic()
                elif count < forwarded:
                    raise RuntimeError('bridge forwarding counter regressed')
                if time.monotonic() - last_advance > 1.5:
                    healthy_since = None
                    reason = 'bridge forwarding did not advance'
                    continue
                if not fresh('flags') or not fresh('local'):
                    healthy_since = None
                    reason = 'PX4 fusion/position telemetry missing'
                    continue
                flags = latest['flags'][1]
                local = latest['local'][1]
                good = (all(flags.values()) and all(local[field] for field in
                    ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control'))
                    and local['dead_reckoning'] is False and count > 0)
                if not good:
                    healthy_since = None
                    reason = 'PX4 external vision or local position/velocity not valid'
                    continue
                if healthy_since is None:
                    healthy_since = time.monotonic()
                if time.monotonic() - healthy_since >= 60:
                    outcome = {'passed': True, 'reason': '60 s bridge/PX4 health window passed',
                               'forwarded': count}
                    break
            else:
                raise RuntimeError(reason + '; 60 s healthy window not completed')
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            outcome = {'passed': False, 'reason': str(exc)}
        finally:
            (run_dir / 'result.json').write_text(
                json.dumps(outcome, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            if node is not None:
                node.destroy_node()
                rclpy.shutdown()
            lock.close()
    print('A mocap recovery: ' + ('PASS' if outcome['passed'] else 'FAIL'), flush=True)
    print(outcome['reason'], flush=True)
    print('Log: ' + str(run_dir), flush=True)
    return 0 if outcome['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
