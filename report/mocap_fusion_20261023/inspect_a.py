"""Read-only A diagnostics: subscribe, request firmware/parameters, capture raw VRPN.
No PARAM_SET, flight command, actuator output or external odometry publishing.
"""
import json
import math
from pathlib import Path
import selectors
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
SERIAL = "1612425621719"


def clean(obj):
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    if isinstance(obj, dict):
        return {str(k): clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if hasattr(obj, "tolist"):
        return clean(obj.tolist())
    return obj


def save(name, data):
    (ROOT / name).write_text(json.dumps(clean(data), indent=2, allow_nan=False), encoding="utf-8")


def backup_parameters():
    from pymavlink import mavutil
    sys.path.insert(0, "/home/nvidia/payload_catch_deploy/current/src/tools")
    from a_solo_preflight import decode_parameter
    link = mavutil.mavlink_connection("udpout:10.41.10.2:14550",
                                     source_system=244, source_component=191)
    result = {"read_only": True, "complete": False}
    try:
        hb = None
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 0)
            msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=.5)
            if msg is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1):
                if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
                    raise RuntimeError("not PX4 A")
                hb = msg.to_dict()
                break
        if hb is None:
            raise RuntimeError("no A FC heartbeat on dedicated ephemeral socket")
        result["heartbeat"] = hb
        link.mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
        version = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            msg = link.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=.5)
            if msg is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1):
                version = msg
                break
        if version is None:
            raise RuntimeError("firmware capabilities unavailable")
        result["version"] = version.to_dict()
        value = int(version.flight_sw_version)
        result["firmware"] = [value >> 24, (value >> 16) & 255, (value >> 8) & 255, value & 255]
        bytewise = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
        ccast = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
        if bytewise == ccast:
            raise RuntimeError("ambiguous parameter encoding")
        result["encoding"] = "bytewise" if bytewise else "c_cast"
        link.mav.param_request_list_send(1, 1)
        params, indices, expected = {}, {}, None
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=.4)
            if msg is None or (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            key = msg.param_id
            if isinstance(key, bytes):
                key = key.decode("ascii")
            key = key.rstrip("\0")
            if key == "_HASH_CHECK":
                continue
            if expected is not None and expected != msg.param_count:
                raise RuntimeError("parameter count changed")
            expected = msg.param_count
            if not 0 <= msg.param_index < expected:
                raise RuntimeError("invalid parameter index")
            val = decode_parameter(msg.param_value, msg.param_type, bytewise)
            entry = {"value": val, "type": msg.param_type, "index": msg.param_index}
            if key in params and params[key] != entry:
                raise RuntimeError("parameter changed during backup: " + key)
            if msg.param_index in indices and indices[msg.param_index] != key:
                raise RuntimeError("parameter index changed")
            params[key], indices[msg.param_index] = entry, key
            if expected and len(indices) == expected:
                break
        result.update(parameters=params, expected=expected, received=len(indices),
                      complete=bool(expected and len(params) == expected
                                    and set(indices) == set(range(expected))))
    except Exception as exc:
        result["error"] = type(exc).__name__ + ": " + str(exc)
    finally:
        link.close()
        save("parameters_backup.json", result)
    return result


def sample_ros_and_raw():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import (VehicleStatus, VehicleLandDetected, VehicleLocalPosition,
                              EstimatorStatusFlags, TimesyncStatus, VehicleOdometry)
    rclpy.init(args=[])
    node = Node("a_mocap_fusion_readonly_inspect", start_parameter_services=False)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    latest, counts, receive_times = {}, {}, {}
    def receive(key, msg):
        latest[key] = {k: clean(getattr(msg, k)) for k in msg.get_fields_and_field_types()}
        counts[key] = counts.get(key, 0) + 1
        receive_times[key] = time.monotonic()
    kinds = [(VehicleStatus, "vehicle_status_v1"),
             (VehicleLandDetected, "vehicle_land_detected"),
             (VehicleLocalPosition, "vehicle_local_position"),
             (EstimatorStatusFlags, "estimator_status_flags"),
             (TimesyncStatus, "timesync_status"), (VehicleOdometry, "vehicle_odometry")]
    subs = [node.create_subscription(cls, "/fmu/out/" + key,
            lambda msg, key=key: receive(key, msg), qos) for cls, key in kinds]
    native = Path("/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0/planeA_stream")
    child = None
    selector = selectors.DefaultSelector()
    raw, buffer = [], b""
    try:
        with (ROOT / "vrpn.stderr.log").open("wb") as err:
            child = subprocess.Popen([str(native), "192.168.1.4"], stdout=subprocess.PIPE,
                                     stderr=err, shell=False)
            os = __import__("os")
            os.set_blocking(child.stdout.fileno(), False)
            selector.register(child.stdout, selectors.EVENT_READ)
            start = time.monotonic()
            while time.monotonic() - start < 10:
                rclpy.spin_once(node, timeout_sec=.003)
                for _, _ in selector.select(timeout=0):
                    block = os.read(child.stdout.fileno(), 65536)
                    if not block:
                        raise RuntimeError("native reader exited: " + str(child.poll()))
                    buffer += block
                    lines = buffer.split(b"\n")
                    buffer = lines.pop()
                    for line in lines:
                        data = json.loads(line)
                        data["arrival_wall_us"] = time.time_ns() // 1000
                        data["arrival_monotonic_ns"] = time.monotonic_ns()
                        raw.append(data)
            topics = node.get_topic_names_and_types()
            relevant = ["/fmu/in/vehicle_visual_odometry", "/fmu/in/vehicle_mocap_odometry",
                        "/fmu/in/vehicle_command", "/fmu/in/offboard_control_mode"]
            endpoints = {}
            for topic in relevant:
                endpoints[topic] = {
                    "publishers": [{"name": i.node_name, "type": i.topic_type}
                                   for i in node.get_publishers_info_by_topic(topic)],
                    "subscribers": [{"name": i.node_name, "type": i.topic_type}
                                    for i in node.get_subscriptions_info_by_topic(topic)]}
            result = {"counts": counts, "latest": latest,
                      "age_s": {k: time.monotonic() - t for k, t in receive_times.items()},
                      "topics": topics, "endpoints": endpoints, "raw_frames": len(raw),
                      "wall_us": time.time_ns() // 1000}
            if raw:
                offsets = [v["arrival_wall_us"] - v["server_stamp_us"] for v in raw]
                result["source_offset_us_range"] = [min(offsets), max(offsets)]
                result["source_stamp_nonincreasing"] = sum(
                    b["server_stamp_us"] <= a["server_stamp_us"] for a, b in zip(raw, raw[1:]))
            save("live_inspection.json", result)
            with (ROOT / "raw_vrpn.jsonl").open("w", encoding="utf-8") as out:
                for row in raw:
                    out.write(json.dumps(row, allow_nan=False) + "\n")
            print(json.dumps(clean(result), allow_nan=False), flush=True)
    finally:
        selector.close()
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=3)
            child.stdout.close()
            save("process_cleanup.json", {"reader_pid": child.pid, "reader_exit": child.returncode})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    serial = Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
    if serial != SERIAL:
        raise RuntimeError("wrong Jetson")
    save("identity.json", {"serial": serial,
                          "boot": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                          "current": str(Path("/home/nvidia/payload_catch_deploy/current").resolve())})
    # The installed rclpy spelling is get_fields_and_field_types.
    sample_ros_and_raw()
    backup = backup_parameters()
    print(json.dumps({"parameter_backup_complete": backup.get("complete"),
                      "parameter_count": backup.get("received"),
                      "firmware": backup.get("firmware"), "error": backup.get("error")}), flush=True)


if __name__ == "__main__":
    main()
