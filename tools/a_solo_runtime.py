#!/usr/bin/env python3
"""Read-only host/service check for A solo; does not inspect/compete with its own ROS publishers."""
import argparse
import json
from pathlib import Path
import socket
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"tools"))
from jetson_runtime import observe
from payload_catch.host_runtime import load_profile,validate_observed


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile",required=True)
    args=ap.parse_args()
    profile=load_profile(args.profile)
    if profile["role"]!="a" or profile["system_id"]!=1 or socket.gethostname()!=profile["hostname"]:
        raise ValueError("A-only runtime profile/host mismatch")
    observed=observe(profile)
    failures=validate_observed(profile,observed)
    print(json.dumps(dict(passed=not failures,failures=failures,
        observations=observed,flight_approval=False,control_commands_sent=0),ensure_ascii=False,indent=2))
    return 1 if failures else 0


if __name__=="__main__":
    raise SystemExit(main())
