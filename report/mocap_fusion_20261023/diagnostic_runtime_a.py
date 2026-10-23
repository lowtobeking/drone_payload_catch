"""A bounded raw+VehicleOdometry diagnostic, isolated from the real DDS agent."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry
from payload_catch.mocap_odometry import load_config
from payload_catch.mocap_transform import Transform


def clean(obj):
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if hasattr(obj, "tolist"):
        return clean(obj.tolist())
    return obj


def main():
    root = Path(__file__).resolve().parent
    logs = Path(sys.argv[1])
    logs.mkdir(parents=True, exist_ok=False)
    if os.environ.get("ROS_DOMAIN_ID") != "191" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("requires isolated localhost domain 191")
    cfg, transform = load_config(root / "config/mocap_a.bridge.yaml")
    assert cfg["enabled"] is False
    assert cfg["server_timing_verified"] is False
    assert cfg["source_to_local_offset_us"] is None
    children, handles = [], []
    node = None
    result = {}
    samples = {"raw": [], "odometry": [], "status": []}
    try:
        for label, argv in [
            ("bridge", [sys.executable, "-m", "payload_catch.mocap_odometry_node",
                        "--config", str(root / "config/mocap_a.bridge.yaml"), "--seconds", "26"]),
            ("source", [sys.executable, "-m", "payload_catch.mocap_source_node",
                        "--reader", str(root / "mocap_vrpn_reader"), "--bind-ip", "192.168.1.4",
                        "--seconds", "25", "--log-dir", str(logs / "source")])]:
            handle = (logs / (label + ".log")).open("wb")
            handles.append(handle)
            proc = subprocess.Popen(argv, cwd=root, stdout=handle,
                                    stderr=subprocess.STDOUT, shell=False)
            children.append((label, proc))
        rclpy.init(args=[])
        node = Node("a_mocap_bridge_diagnostic_capture", start_parameter_services=False)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        def take(key, msg):
            data = (json.loads(msg.data) if key != "odometry"
                    else {k: clean(getattr(msg, k)) for k in msg.get_fields_and_field_types()})
            samples[key].append(data)
        subs = [
            node.create_subscription(String, "/payload/mocap/a/raw_sample", lambda m: take("raw", m), qos),
            node.create_subscription(VehicleOdometry, "/payload/mocap/a/odometry_preview",
                                     lambda m: take("odometry", m), qos),
            node.create_subscription(String, "/payload/mocap/a/bridge_status",
                                     lambda m: take("status", m), qos)]
        start = time.monotonic()
        while time.monotonic() - start < 18:
            rclpy.spin_once(node, timeout_sec=.005)
            if any(p.poll() is not None for _, p in children):
                raise RuntimeError("diagnostic child exited early")
        for topic, _ in node.get_topic_names_and_types():
            if "/fmu/in/" in topic and node.get_publishers_info_by_topic(topic):
                raise RuntimeError("unexpected FC input publisher in isolated diagnostic")
        raw_by_stamp = {d["receive_wall_us"]: d for d in samples["raw"]}
        matched, max_error = 0, 0.
        for odo in samples["odometry"]:
            raw = raw_by_stamp.get(odo["timestamp_sample"])
            if raw is None:
                continue
            expected, q = transform.convert(raw["position"], raw["quaternion_xyzw"])
            max_error = max(max_error, max(abs(a-b) for a,b in zip(expected, odo["position"])))
            if abs(abs(sum(a*b for a,b in zip([q[3], *q[:3]], odo["q"]))) - 1) > 1e-5:
                raise RuntimeError("quaternion conversion mismatch")
            if odo["pose_frame"] != 2 or any(x is not None for x in odo["velocity"]):
                raise RuntimeError("bad frame or invented velocity")
            matched += 1
        if max_error > 1e-5:
            raise RuntimeError("position conversion mismatch")
        latest = samples["status"][-1] if samples["status"] else {}
        result = dict(passed=bool(len(samples["raw"]) > 500 and len(samples["odometry"]) > 100
                                  and matched > 100 and not latest.get("latched_fault", "missing")),
                      sample_seconds=18, counts={k: len(v) for k,v in samples.items()},
                      matched_positions=matched, max_position_arithmetic_error_m=max_error,
                      final_status=latest, fmu_input_publishers=0, forwarded_to_px4=False,
                      server_timing_verified=False, flight_approval=False)
        (logs / "validation.json").write_text(json.dumps(clean(result), indent=2, allow_nan=False),
                                              encoding="utf-8")
        print(json.dumps(clean(result), allow_nan=False), flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cleanup = []
        for label, proc in reversed(children):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
            cleanup.append({"name": label, "pid": proc.pid, "exit_code": proc.returncode})
        for handle in handles:
            handle.close()
        (logs / "process_cleanup.json").write_text(json.dumps(cleanup, indent=2), encoding="utf-8")
        with (logs / "samples.jsonl").open("w", encoding="utf-8") as handle:
            for key, rows in samples.items():
                for data in rows:
                    handle.write(json.dumps({"kind": key, "data": clean(data)}, allow_nan=False) + "\n")
    return 0 if result.get("passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
