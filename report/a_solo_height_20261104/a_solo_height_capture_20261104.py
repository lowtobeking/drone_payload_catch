"""Read-only, bounded capture of A PX4 local position and solo task state."""
import argparse
import json
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=90)
    parser.add_argument("--height", type=float, default=0.2)
    args = parser.parse_args()
    rclpy.init()
    node = Node("a_solo_height_capture")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    origin = None
    phase = None
    count = 0

    def emit(row):
        print(json.dumps(row, separators=(",", ":"), allow_nan=False), flush=True)

    def state(msg):
        nonlocal origin, phase
        data = json.loads(msg.data)
        if data.get("scope") != "a_solo_no_payload_v1":
            return
        origin = data.get("origin")
        phase = data.get("phase")
        emit({"kind": "state", "receive_wall_s": time.time(), "data": data})

    def task(msg):
        data = json.loads(msg.data)
        if data.get("scope") == "a_solo_no_payload_v1":
            emit({"kind": "task", "receive_wall_s": time.time(), "data": data})

    def position(msg):
        nonlocal count
        count += 1
        row = {"kind": "position", "receive_wall_s": time.time(),
               "px4_timestamp_us": msg.timestamp, "phase": phase,
               "x_m": msg.x, "y_m": msg.y, "z_ned_m": msg.z,
               "vx_m_s": msg.vx, "vy_m_s": msg.vy, "vz_down_m_s": msg.vz,
               "xy_valid": msg.xy_valid, "z_valid": msg.z_valid,
               "v_xy_valid": msg.v_xy_valid, "v_z_valid": msg.v_z_valid}
        if origin is not None and len(origin) == 3:
            row["target_z_ned_m"] = origin[2] - args.height
            row["relative_height_m"] = origin[2] - msg.z
            row["height_error_m"] = row["relative_height_m"] - args.height
            row["vertical_velocity_up_m_s"] = -msg.vz
        emit(row)

    node.create_subscription(String, "/a_validation/state", state, 10)
    node.create_subscription(String, "/a_validation/task", task, 10)
    node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", position, qos)
    emit({"kind": "capture_start", "receive_wall_s": time.time(), "height_m": args.height,
          "duration_s": args.seconds})
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        emit({"kind": "capture_end", "receive_wall_s": time.time(), "position_count": count})
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
