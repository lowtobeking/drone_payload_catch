#!/usr/bin/env python3
"""Logged A-only preflight and optional single-attempt flight orchestration."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.solo_validation import SCOPE, code_digest, load_config


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def preflight(config, profile, run_dir):
    receipt = run_dir / "preflight.json"
    command = [sys.executable, str(ROOT / "tools/a_solo_preflight.py"),
               "--config", str(config), "--host-profile", str(profile),
               "--mavlink", "passive:eno1", "--out", str(receipt)]
    with (run_dir / "preflight.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                timeout=110, shell=False)
    report = json.loads(receipt.read_text(encoding="utf-8")) if receipt.exists() else {}
    passed = result.returncode == 0 and report.get("passed") is True
    print("A preflight: " + ("PASS" if passed else "FAIL"), flush=True)
    for issue in report.get("issues", []):
        print("  " + str(issue), flush=True)
    if not passed and not report.get("issues"):
        print("  See " + str(run_dir / "preflight.log"), flush=True)
    if not passed:
        for row in report.get("parameter_checks", []):
            if row.get("level") == "fail":
                print("  " + row["name"] + ": " + row.get("detail", ""), flush=True)
    print("Preflight log: " + str(run_dir / "preflight.log"), flush=True)
    print("Receipt: " + str(receipt), flush=True)
    return passed, receipt


def finite(value):
    return float(value) if math.isfinite(value) else None


def fly(config, profile, receipt, run_dir):
    import rclpy
    from rclpy.node import Node
    from px4_msgs.msg import VehicleLocalPosition
    from std_msgs.msg import String
    from payload_catch.px4_iface import qos_in
    from payload_catch.solo_validation_node import TOPIC, bus_qos

    cfg = load_config(config)
    fingerprint = code_digest(ROOT)
    rclpy.init(args=[])
    node = Node("a_solo_auto_observer")
    latest = None
    received = 0.0
    mission = None
    result = {"outcome": "not_started", "run_dir": str(run_dir)}
    with (run_dir / "state_position.jsonl").open("w", encoding="utf-8", buffering=1) as trace, \
         (run_dir / "mission.log").open("w", encoding="utf-8", buffering=1) as mission_log:
        def record(kind, data):
            trace.write(json.dumps({"kind": kind, "received_at": time.time(), "data": data},
                                   ensure_ascii=False, allow_nan=False) + "\n")

        def state(msg):
            nonlocal latest, received
            data = json.loads(msg.data)
            if (data.get("scope") != SCOPE or data.get("config_digest") != cfg.digest
                    or data.get("code_digest") != fingerprint):
                return
            latest, received = data, time.monotonic()
            record("state", data)

        def task(msg):
            data = json.loads(msg.data)
            if data.get("scope") == SCOPE and data.get("config_digest") == cfg.digest:
                record("task", data)

        def position(msg):
            row = {"px4_timestamp_us": msg.timestamp,
                   "x_m": finite(msg.x), "y_m": finite(msg.y), "z_ned_m": finite(msg.z),
                   "vx_m_s": finite(msg.vx), "vy_m_s": finite(msg.vy),
                   "vz_down_m_s": finite(msg.vz), "xy_valid": msg.xy_valid,
                   "z_valid": msg.z_valid, "v_xy_valid": msg.v_xy_valid,
                   "v_z_valid": msg.v_z_valid,
                   "phase": latest.get("phase") if latest else None}
            if latest and latest.get("origin") and row["z_ned_m"] is not None:
                row["target_z_ned_m"] = latest["origin"][2] - cfg.height_m
                row["height_m"] = latest["origin"][2] - row["z_ned_m"]
                row["height_error_m"] = row["height_m"] - cfg.height_m
            record("position", row)

        node.create_subscription(String, TOPIC + "state", state, bus_qos())
        node.create_subscription(String, TOPIC + "task", task, bus_qos())
        node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 position, qos_in())
        command = ["/bin/bash", str(ROOT / "deploy/run_a_solo.sh"), str(config),
                   str(receipt), str(profile), "execute"]
        mission = subprocess.Popen(command, cwd=ROOT, stdout=mission_log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        result["mission_pid"] = mission.pid
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if mission.poll() is not None:
                    raise RuntimeError("mission exited before READY")
                if (latest and latest.get("phase") == "READY" and latest.get("execute") is True
                        and time.monotonic() - received < .5):
                    break
            else:
                raise RuntimeError("READY timeout: " + (latest.get("reason", "no state") if latest else "no state"))

            result["outcome"] = "start_requested"
            with (run_dir / "start.log").open("w", encoding="utf-8") as start_log:
                start = subprocess.run([sys.executable, str(ROOT / "tools/a_solo_control.py"),
                    "start", "--confirm", "A_ONLY_TAKEOFF"], cwd=ROOT,
                    stdout=start_log, stderr=subprocess.STDOUT, timeout=12, shell=False)
            if start.returncode:
                result["outcome"] = "intervention_required"
                raise RuntimeError("START not confirmed; inspect state and start.log before any retry")
            result["outcome"] = "started"
            print("A START confirmed; recording flight", flush=True)
            deadline = time.monotonic() + cfg.max_flight_s + 45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if latest and time.monotonic() - received < .5:
                    if latest.get("phase") == "DONE":
                        result["final_state"] = latest
                        result["outcome"] = ("passed" if latest.get("reason") == "LAND_REQUESTED"
                            and latest.get("progress") == "LANDED_DISARMED_CONFIRMED" else "failed")
                        break
                    if latest.get("phase") in ("FAILED", "MANUAL", "KILL"):
                        result["final_state"] = latest
                        result["outcome"] = "intervention_required"
                        break
            else:
                result["outcome"] = "intervention_required"
                result["error"] = "terminal state not confirmed before deadline"
        except (KeyboardInterrupt, RuntimeError, subprocess.TimeoutExpired) as exc:
            result["error"] = str(exc)
            if result["outcome"] in ("started", "start_requested"):
                result["outcome"] = "intervention_required"
        finally:
            # Stop only after confirmed disarm, or before START was accepted.
            if result["outcome"] in ("passed", "failed", "not_started") and mission.poll() is None:
                os.killpg(mission.pid, signal.SIGTERM)
                try:
                    mission.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    result["error"] = "mission process did not stop; inspect its process group"
            if mission.poll() is None:
                result["mission_still_running"] = True
            save(run_dir / "result.json", result)
            node.destroy_node()
            rclpy.shutdown()
    print("A mission: " + result["outcome"].upper(), flush=True)
    if result.get("error"):
        print("Reason: " + result["error"], flush=True)
    elif result.get("final_state"):
        print("Reason: " + result["final_state"].get("reason", ""), flush=True)
    print("Flight log: " + str(run_dir / "state_position.jsonl"), flush=True)
    print("Result: " + str(run_dir / "result.json"), flush=True)
    if result["outcome"] == "intervention_required":
        print("Use independent RC/PX4; mission process left running for safety.", flush=True)
    return 0 if result["outcome"] == "passed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "fly"))
    parser.add_argument("--base", default="/home/nvidia/payload_catch_deploy")
    args = parser.parse_args()
    base = Path(args.base).resolve()
    config = base / "a_solo.yaml"
    profile = base / "host/runtime.yaml"
    logs = base / "flight_logs"
    logs.mkdir(parents=True, exist_ok=True)
    run_dir = logs / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_")
                      + args.mode + "_" + uuid.uuid4().hex[:8])
    run_dir.mkdir()
    print("Run: " + str(run_dir), flush=True)
    try:
        passed, receipt = preflight(config, profile, run_dir)
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        save(run_dir / "result.json", {"outcome": "preflight_error", "error": str(exc)})
        print("A preflight: FAIL - " + str(exc), flush=True)
        return 1
    if not passed:
        save(run_dir / "result.json", {"outcome": "preflight_failed", "receipt": str(receipt)})
        return 1
    if args.mode == "check":
        save(run_dir / "result.json", {"outcome": "preflight_passed", "receipt": str(receipt)})
        return 0
    return fly(config, profile, receipt, run_dir)


if __name__ == "__main__":
    raise SystemExit(main())
