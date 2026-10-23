"""Bounded planeA native reader -> raw timing envelope. No FC input publisher."""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


def run(reader, server, bind_ip, seconds, log_dir):
    rclpy.init(args=[])
    node = Node("a_mocap_raw_source", start_parameter_services=False)
    pub = node.create_publisher(String, "/payload/mocap/a/raw_sample",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE))
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    child = None
    selector = selectors.DefaultSelector()
    buffer = b""
    stop = False
    published = 0
    def stopping(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGTERM, stopping)
    signal.signal(signal.SIGINT, stopping)
    try:
        with (log_dir / "native.stderr.log").open("wb") as err:
            child = subprocess.Popen([str(Path(reader).resolve()), server, "planeA", bind_ip,
                                      str(seconds)], stdout=subprocess.PIPE, stderr=err, shell=False)
            (log_dir / "reader_process.json").write_text(json.dumps({
                "pid": child.pid, "reader": str(reader), "purpose": "planeA bounded raw source"}),
                encoding="utf-8")
            os.set_blocking(child.stdout.fileno(), False)
            selector.register(child.stdout, selectors.EVENT_READ)
            start = time.monotonic()
            while not stop and rclpy.ok() and time.monotonic() - start < seconds:
                for _, _ in selector.select(timeout=.005):
                    block = os.read(child.stdout.fileno(), 65536)
                    if not block:
                        raise RuntimeError("native reader exited: " + str(child.poll()))
                    buffer += block
                    if len(buffer) > 262144:
                        raise RuntimeError("native queue overflow; restart required")
                    lines = buffer.split(b"\n")
                    buffer = lines.pop()
                    # Do not drain an old batch into PX4 as a burst of new poses.
                    # Old samples are discarded here and freshness is checked downstream.
                    if lines:
                        data = json.loads(lines[-1])
                        if (data.get("schema"), data.get("tracker"), data.get("sensor")) != (1, "planeA", 0):
                            raise RuntimeError("invalid native source identity")
                        msg = String()
                        msg.data = json.dumps(data, allow_nan=False)
                        pub.publish(msg)
                        published += 1
                rclpy.spin_once(node, timeout_sec=0.)
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
            (log_dir / "reader_cleanup.json").write_text(json.dumps({
                "pid": child.pid, "exit_code": child.returncode, "published": published}),
                encoding="utf-8")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--reader", required=True)
    ap.add_argument("--server", default="192.168.1.111:3883")
    ap.add_argument("--bind-ip", required=True)
    ap.add_argument("--seconds", type=float, default=30.)
    ap.add_argument("--log-dir", required=True)
    args = ap.parse_args()
    if not 1 <= args.seconds <= 120:
        ap.error("--seconds must be within 1..120")
    run(args.reader, args.server, args.bind_ip, args.seconds, args.log_dir)


if __name__ == "__main__":
    main()
