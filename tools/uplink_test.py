#!/usr/bin/env python3
"""上行链路验证（参考 drone_package_20260908/tools/uplink_test.py）。

只发 `OffboardControlMode` **心跳**（不 ARM、不发 setpoint、不起飞），
订阅 `failsafe_flags.offboard_control_signal_lost`：若它从 true 翻到 false，
说明 PX4 收到了我们的 offboard 流（上行通）。

    python3 tools/uplink_test.py                 # drone 0，5s
    python3 tools/uplink_test.py --drone 1 --seconds 5
    python3 tools/uplink_test.py --json

判定逻辑 `verdict(flags)` 是纯函数，`tools/test_uplink_test.py` 离线单测。
"""
from __future__ import annotations

import argparse
import json
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from px4_msgs.msg import FailsafeFlags, OffboardControlMode
    HAVE_ROS = True
except Exception:                                             # noqa: BLE001
    HAVE_ROS = False


def _prefix(drone_id: int) -> str:
    return '/fmu' if drone_id == 0 else f'/px4_{drone_id}/fmu'


def verdict(flags) -> tuple:
    """flags: 窗口内采样到的 offboard_control_signal_lost 列表。

    至少一次 False（收到 offboard 流）才算通过。
    """
    if not flags:
        return False, '未收到 failsafe_flags（订阅未通？）'
    lost = sum(1 for f in flags if f)
    ok = lost < len(flags)
    return ok, f'lost {lost}/{len(flags)}'


if HAVE_ROS:
    def _qos_in():
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _qos_out():
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5,
                          durability=DurabilityPolicy.VOLATILE)

    class _Node(Node):
        def __init__(self, drone_id):
            super().__init__('uplink_test')
            self.flags = []
            p = _prefix(drone_id)
            self.create_subscription(FailsafeFlags, f'{p}/out/failsafe_flags',
                                     self._on_ff, _qos_in())
            self.pub = self.create_publisher(OffboardControlMode,
                                             f'{p}/in/offboard_control_mode', _qos_out())
            self.create_timer(0.05, self._tick)      # 20 Hz 心跳

        def _on_ff(self, m):
            self.flags.append(bool(m.offboard_control_signal_lost))

        def _tick(self):
            m = OffboardControlMode()
            m.position = False
            m.velocity = True
            m.acceleration = False
            m.attitude = False
            m.body_rate = False
            m.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub.publish(m)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='offboard 上行链路验证（只发心跳）')
    ap.add_argument('--drone', type=int, default=0, choices=[0, 1])
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    if not HAVE_ROS:
        print('❌ uplink_test 需要 ROS（rclpy/px4_msgs）；请先 source env.sh')
        return 2

    rclpy.init()
    node = _Node(args.drone)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    ok, detail = verdict(node.flags)
    node.destroy_node()
    rclpy.shutdown()

    if args.json:
        print(json.dumps({'ok': ok, 'detail': detail, 'samples': len(node.flags)},
                         ensure_ascii=False))
    else:
        print(f'{"✅" if ok else "❌"} uplink drone{args.drone}: {detail}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
