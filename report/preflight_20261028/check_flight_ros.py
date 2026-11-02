#!/usr/bin/env python3
"""Isolated ROS check: armed continuity and mocap loss, never real FC."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time

if os.environ.get("ROS_DOMAIN_ID") != "192" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("isolated localhost ROS domain 192 required")
root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root))
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLandDetected, TimesyncStatus, EstimatorStatusFlags
from std_msgs.msg import String
from payload_catch.mocap_odometry_node import MocapOdometry

cfg = yaml.safe_load((root / "config/mocap_a.solo_flight.yaml").read_text(encoding="utf-8"))
cfg.update(enabled=True, mounting_confirmed=True, lever_arm_measured=True,
           tracking_loss_verified=True)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
with tempfile.TemporaryDirectory() as temporary:
    config_path = Path(temporary) / "flight.yaml"
    config_path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    rclpy.init(args=[])
    driver = Node("flight_bridge_isolated_driver", start_parameter_services=False)
    raw = driver.create_publisher(String, "/payload/mocap/a/raw_sample", qos)
    status = driver.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", qos)
    land = driver.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", qos)
    sync = driver.create_publisher(TimesyncStatus, "/fmu/out/timesync_status", qos)
    flags = driver.create_publisher(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", qos)
    bridge = MocapOdometry(config_path, send_to_px4=True, isolated_test=True,
                           flight_receive_time=True)
    def phase(seconds, *, armed=False, samples=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            wall = time.time_ns() // 1000
            mono = time.monotonic_ns() // 1000
            s = VehicleStatus()
            s.system_id, s.component_id, s.arming_state = 1, 1, 2 if armed else 1
            status.publish(s)
            l = VehicleLandDetected()
            l.landed = not armed
            land.publish(l)
            t = TimesyncStatus()
            t.source_protocol, t.remote_timestamp = 2, wall - 100
            t.round_trip_time, t.estimated_offset = 500, 1000
            sync.publish(t)
            f = EstimatorStatusFlags()
            f.cs_ev_pos = f.cs_ev_yaw = f.cs_ev_hgt = True
            flags.publish(f)
            if samples:
                m = String()
                m.data = json.dumps(dict(schema=1, tracker="planeA", sensor=0, frame="world",
                    server_stamp_us=wall + 43_200_000_000,
                    receive_wall_us=wall, receive_monotonic_us=mono,
                    position=[1.0 + mono * 1e-9, 2.0, 3.0],
                    quaternion_xyzw=[0.0, 0.0, 1.0, 0.0]))
                raw.publish(m)
            rclpy.spin_once(driver, timeout_sec=.002)
            rclpy.spin_once(bridge, timeout_sec=.002)
            time.sleep(.012)
    try:
        phase(1.2)
        before = bridge.published
        assert before > 5 and not bridge.fixed_fault, (before, bridge.reason)
        phase(.5, armed=True)
        after_arm = bridge.published
        assert after_arm > before and not bridge.ground.fault, (before, after_arm, bridge.reason)
        phase(.3, armed=True, samples=False)
        stopped = bridge.published
        assert bridge.builder.fault and stopped == bridge.published
        phase(.2, armed=True)
        assert bridge.published == stopped
        print(json.dumps({"passed": True, "disarmed_forwarded": before,
                          "armed_forwarded": after_arm - before,
                          "loss_latched": bridge.builder.fault,
                          "no_auto_resume": bridge.published == stopped}))
    finally:
        bridge.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
