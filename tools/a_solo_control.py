#!/usr/bin/env python3
"""Operator requests for A-only acceptance. start and kill require explicit phrases."""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=["status", "start", "stop", "kill"])
    ap.add_argument("--confirm", default="")
    args = ap.parse_args()
    expected = {"start":"A_ONLY_TAKEOFF", "kill":"CUT_MOTOR_POWER"}
    if args.action in expected and args.confirm != expected[args.action]:
        ap.error("requires --confirm "+expected[args.action])
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from payload_catch.solo_validation import SCOPE
    from payload_catch.solo_validation_node import bus_qos, TOPIC
    rclpy.init(args=[])
    node = Node("a_solo_operator_"+uuid.uuid4().hex[:8])
    latest = {}
    def receive(msg):
        nonlocal latest
        data = json.loads(msg.data)
        if data.get("scope") == SCOPE and 0 <= time.time()-data.get("stamp", 0) <= .5:
            latest = data
    node.create_subscription(String, TOPIC+"state", receive, bus_qos())
    publisher = node.create_publisher(String, TOPIC+"request", bus_qos())
    try:
        deadline = time.monotonic()+4
        while not latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not latest:
            raise RuntimeError("no fresh A-only supervisor; use independent RC")
        print(json.dumps(latest, ensure_ascii=False, indent=2))
        if args.action == "status":
            return 0
        if args.action == "start" and (latest.get("phase") != "READY" or latest.get("execute") is not True):
            raise RuntimeError("A not READY: "+latest.get("reason", ""))
        request = {key:latest[key] for key in ("scope","session","config_digest","code_digest")}
        request.update(id=uuid.uuid4().hex, stamp=time.time(), action=args.action, confirm=args.confirm,
                       reason="operator "+args.action)
        initial_seq = latest["seq"]
        # Repeated identical ID covers discovery/loss but cannot execute twice.
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            if time.time()-request["stamp"] <= .45:
                publisher.publish(String(data=json.dumps(request)))
            rclpy.spin_once(node, timeout_sec=.03)
            accepted = (latest["seq"] > initial_seq and
                (latest["phase"] not in ("READY","CHECK") if args.action == "start"
                 else latest["phase"] in ("LAND","CANCELLED","MANUAL","KILL","DONE","FAILED")))
            if accepted:
                print("observed supervisor: "+latest["phase"]+"; "+latest["reason"])
                return 0
        raise RuntimeError("request not confirmed; inspect A state / use RC")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
