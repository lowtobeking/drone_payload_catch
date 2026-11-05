#!/usr/bin/env python3
"""Live A-only acceptance receipt. Reads PX4 parameters; never writes or arms."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.solo_validation import SCOPE, load_config, code_digest, preflight_errors, parameter_issues


def decode_parameter(value, kind, bytewise):
    """PX4 integer PARAM_VALUE is a typed byte union, NOT its raw float value."""
    if kind == 9:
        result = float(value)
    elif kind in (1, 2, 3, 4, 5, 6):
        if bytewise:
            fmt = {1:"B", 2:"b", 3:"H", 4:"h", 5:"I", 6:"i"}[kind]
            result = struct.unpack("<"+fmt, struct.pack("<f", value)[:struct.calcsize(fmt)])[0]
        else:
            if not math.isfinite(value) or value != int(value):
                raise ValueError("invalid C-cast integer parameter")
            result = int(value)
    else:
        raise ValueError("unsupported parameter type "+str(kind))
    if not math.isfinite(result):
        raise ValueError("nonfinite parameter")
    return result


def live_parameters(connection):
    from pymavlink import mavutil
    link = mavutil.mavlink_connection(connection, source_system=254, source_component=190)
    try:
        hb = None
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 0)
            msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is not None:
                # PX4 can forward the GCS heartbeat from its other MAVLink port.
                # GCS traffic is not an autopilot identity; still reject foreign FCs.
                if (msg.type == mavutil.mavlink.MAV_TYPE_GCS
                        and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                    continue
                if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1 or msg.autopilot != 12:
                    raise RuntimeError("connection is not PX4 A SYSID1/component1")
                hb = msg
                break
        if hb is None:
            raise RuntimeError("A heartbeat missing")
        # Request capability bits before decoding any integer parameters.
        link.mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
        version = None
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            msg = link.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=.5)
            if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                version = msg
                break
        if version is None:
            raise RuntimeError("parameter encoding capability unknown")
        bytewise = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
        ccast = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
        if bytewise == ccast:
            raise RuntimeError("ambiguous / unsupported parameter encoding")
        link.mav.param_request_list_send(1, 1)
        parameters, indices, expected = {}, {}, None
        deadline = time.monotonic()+45
        while time.monotonic() < deadline:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=.5)
            if msg is None or msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                continue
            if expected is not None and msg.param_count != expected:
                raise RuntimeError("parameter count changed")
            expected = msg.param_count
            key = msg.param_id
            if isinstance(key, bytes):
                key = key.decode("ascii")
            key = key.rstrip("\0")
            # PX4 prefixes PARAM_REQUEST_LIST with a cache hash. This is
            # metadata, not an entry in the indexed parameter snapshot.
            if key == "_HASH_CHECK":
                if msg.param_index != 65535 or msg.param_type != 5:
                    raise RuntimeError("invalid PX4 parameter hash metadata")
                continue
            if not 0 <= msg.param_index < expected:
                raise RuntimeError("parameter index invalid")
            value = decode_parameter(msg.param_value, msg.param_type, bytewise)
            if ((msg.param_index in indices and indices[msg.param_index] != key)
                    or (key in parameters and parameters[key] != value)):
                raise RuntimeError("parameter snapshot changed")
            indices[msg.param_index], parameters[key] = key, value
            if expected and len(indices) == expected:
                break
        if not expected or set(indices) != set(range(expected)) or len(parameters) != expected:
            raise RuntimeError("parameter list incomplete")
        return parameters
    finally:
        link.close()


def write_report(path, report):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name+".writing")
    staging.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    staging.replace(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--host-profile", required=True)
    ap.add_argument("--mavlink", required=True, help="dedicated A read endpoint; no parameter file approvals")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    # Invalidate an older approval BEFORE any potentially failing operation.
    write_report(args.out, dict(schema=SCOPE, passed=False, checked_at=time.time(),
                               issues=["preflight in progress"]))
    errors, parameters, rows = [], {}, []
    try:
        cfg = load_config(args.config)
        from payload_catch.solo_validation_node import AHealth, local_identity, check_runtime
        identity = local_identity()
        errors += cfg.site_errors(identity)
        errors += check_runtime(ROOT, args.host_profile)
        if args.mavlink == "passive:eno1":
            import os
            import subprocess
            env = dict(os.environ, PYTHONPATH=str(ROOT) + ":" + os.environ.get("PYTHONPATH", ""))
            proc = subprocess.run(["sudo", "-n", "/usr/bin/env",
                "PYTHONPATH=" + env["PYTHONPATH"],
                "/usr/bin/python3", str(ROOT / "tools/a_solo_parameters_passive.py")],
                cwd=ROOT, timeout=65, capture_output=True, text=True,
                encoding="utf-8", errors="replace", shell=False)
            if proc.returncode:
                raise RuntimeError("passive A parameter read failed: " + proc.stderr.strip())
            parameters = json.loads(proc.stdout)
        else:
            parameters = live_parameters(args.mavlink)
        rows = parameter_issues(parameters, cfg)
        errors += [r["name"]+": "+r["detail"] for r in rows if r["level"] == "fail"]
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=[])
        node = Node("a_solo_readonly_preflight")
        try:
            health = AHealth(node)
            deadline, good_since, live_errors = time.monotonic()+8, None, ["no A telemetry"]
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
                snapshot, telemetry = health.observe()
                live_errors = preflight_errors(snapshot, telemetry, cfg)
                if live_errors:
                    good_since = None
                elif good_since is None:
                    good_since = time.monotonic()
                elif time.monotonic()-good_since >= 3:
                    break
            errors += live_errors
            if good_since is None or time.monotonic()-good_since < 3:
                errors.append("A live health did not pass continuously for 3 seconds")
        finally:
            node.destroy_node()
            rclpy.shutdown()
        report = dict(schema=SCOPE, scope=SCOPE, passed=not errors, checked_at=time.time(),
            drone_ids=[0], system_id=1, battery_min=.4, rc=True, gps=cfg.gps,
            release_allowed=False, config_digest=cfg.digest, code_digest=code_digest(ROOT),
            identity=identity, params_checked=[0] if not errors else [],
            param_profile="outdoor" if cfg.gps else "indoor",
            params_sha256=hashlib.sha256(json.dumps(parameters, sort_keys=True,
                allow_nan=False).encode()).hexdigest(), issues=errors, parameter_checks=rows)
    except Exception as exc:
        report = dict(schema=SCOPE, passed=False, checked_at=time.time(),
            issues=errors+[type(exc).__name__+": "+str(exc)])
    write_report(args.out, report)
    if parameters:
        write_report(str(args.out)+".params.json", parameters)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
