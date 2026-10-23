"""Read-only capture in isolated preview DDS domain; publishes no messages."""
import json
import math
import os
from pathlib import Path
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

if os.environ.get("ROS_DOMAIN_ID") != "191" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("isolated localhost domain required")
rclpy.init()
node = Node("a_mocap_preview_capture", start_parameter_services=False)
counts = {"raw": 0, "preview": 0, "status": 0}
last = {}
ages = []
started = time.monotonic()
output_dir = Path(sys.argv[1])
stream = (output_dir / "poses.jsonl").open("w", encoding="utf-8")

def receive_pose(kind, msg):
    p, q = msg.pose.position, msg.pose.orientation
    v = dict(frame=msg.header.frame_id, stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
             position=[p.x,p.y,p.z], quaternion_xyzw=[q.x,q.y,q.z,q.w])
    if kind == "preview":
        v["roll_pitch_yaw_deg"] = Rotation.from_quat(v["quaternion_xyzw"]).as_euler("xyz", degrees=True).tolist()
    counts[kind] += 1
    last[kind] = v
    stream.write(json.dumps(dict(kind=kind, elapsed_s=time.monotonic()-started, **v),allow_nan=False)+"\n")

def receive_status(msg):
    counts["status"] += 1
    last["status"] = json.loads(msg.data)

subscriptions = [
    node.create_subscription(PoseStamped, "/vrpn_mocap/planeA/pose", lambda m: receive_pose("raw",m), QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)),
    node.create_subscription(PoseStamped, "/payload/mocap/a/preview_pose", lambda m: receive_pose("preview",m), 10),
    node.create_subscription(String, "/payload/mocap/a/preview_status", receive_status, 10)]
try:
    while time.monotonic()-started < 18:
        rclpy.spin_once(node, timeout_sec=.04)
    fmu_publishers = {t: len(node.get_publishers_info_by_topic(t)) for t,_ in node.get_topic_names_and_types() if "/fmu/in/" in t}
    result = dict(counts=counts, last=last, elapsed_s=time.monotonic()-started,
                  isolated_domain=191, fmu_publishers=fmu_publishers,
                  topics=node.get_topic_names_and_types(), nodes=node.get_node_names_and_namespaces(),
                  forwarded_to_px4=False)
    result["passed"] = (counts["raw"]>100 and counts["preview"]>100 and counts["status"]>3
                        and not last["status"]["stale"] and not any(fmu_publishers.values()))
    (output_dir/"capture.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
    if not result["passed"]:
        raise SystemExit(2)
finally:
    stream.close()
    node.destroy_node()
    rclpy.shutdown()
