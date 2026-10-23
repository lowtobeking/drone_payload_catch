#!/usr/bin/env python3
"""Bounded synthetic ROS preview check. Localhost-only and isolated domain."""
import json
import math
import os
from pathlib import Path
import sys
import time

if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "191":
    raise SystemExit("requires isolated ROS domain 191 and localhost-only")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from payload_catch.mocap_preview_node import MocapPreview

rclpy.init()
node = None
driver = None
try:
    node = MocapPreview(ROOT / "config/mocap_a.preview.yaml")
    driver = Node("mocap_preview_isolated_test", start_parameter_services=False)
    pub = driver.create_publisher(PoseStamped, "/vrpn_mocap/planeA/pose", 5)
    poses, statuses = [], []
    sub_pose = driver.create_subscription(PoseStamped, "/payload/mocap/a/preview_pose", poses.append, 5)
    sub_status = driver.create_subscription(String, "/payload/mocap/a/preview_status", lambda m: statuses.append(json.loads(m.data)), 5)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            rclpy.spin_once(driver, timeout_sec=.01)

    def pose():
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = driver.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 1., 2., 3.
        m.pose.orientation.z = 1.
        m.pose.orientation.w = 0.
        return m

    spin(.8)
    m = pose()
    for _ in range(6):
        m = pose()
        pub.publish(m)
        spin(.08)
    if not poses:
        raise AssertionError("no preview output")
    out = poses[-1]
    assert [out.pose.position.x, out.pose.position.y, out.pose.position.z] == [2., 1., -3.]
    assert abs(abs(out.pose.orientation.w) - 1.) < 1e-6
    count = node.accepted

    # Same timestamp must not extend fresh-data state.
    pub.publish(m)
    spin(.15)
    assert node.accepted == count and node.rejected >= 1
    m = pose()
    m.pose.position.x = math.nan
    pub.publish(m)
    spin(.15)
    assert node.accepted == count
    m = pose()
    m.header.frame_id = "wrong_world"
    pub.publish(m)
    spin(.15)
    assert node.accepted == count
    m = pose()
    m.header.stamp.sec -= 1
    pub.publish(m)
    spin(.15)
    assert node.accepted == count
    spin(1.3)
    assert statuses and statuses[-1]["stale"]
    assert not statuses[-1]["forwarded_to_px4"]
    assert not statuses[-1]["flight_approval"]
    for topic, _ in node.get_topic_names_and_types():
        if topic.startswith("/fmu/in/") or topic.startswith("/px4_1/fmu/in/"):
            assert not node.get_publishers_info_by_topic(topic), topic
    print(json.dumps(dict(passed=True,synthetic=True,domain=191,
        accepted=node.accepted,rejected=node.rejected,
        stale_status=statuses[-1]["stale"],fmu_publishers=0,flight_approval=False)))
finally:
    if driver is not None:
        driver.destroy_node()
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
