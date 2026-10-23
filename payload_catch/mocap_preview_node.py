"""A-only mocap diagnostic preview. Never publishes any /fmu/in topic."""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .mocap_transform import StampGate, load_config


class MocapPreview(Node):
    def __init__(self, config):
        super().__init__("a_mocap_preview", start_parameter_services=False)
        if self.get_parameter("use_sim_time").value:
            raise ValueError("hardware preview requires real local ROS time")
        self.config, self.transform = load_config(config)
        self.gate = StampGate(self.config["max_local_stamp_age_s"])
        self.accepted = self.rejected = 0
        self.last_good = None
        self.last_error = "waiting for planeA"
        self.pose_out = self.create_publisher(PoseStamped, "/payload/mocap/a/preview_pose", 5)
        self.status_out = self.create_publisher(String, "/payload/mocap/a/preview_status", 5)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            PoseStamped, self.config["input_topic"], self.receive, qos)
        self.create_timer(1., self.report)
        self.get_logger().warn("PREVIEW ONLY: candidate mounting; no PX4 input publisher")

    def receive(self, msg):
        try:
            if msg.header.frame_id != self.config["input_frame"]:
                raise ValueError("unexpected mocap frame_id: " + msg.header.frame_id)
            p, q = msg.pose.position, msg.pose.orientation
            pos, quat = self.transform.convert([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.gate.check(stamp, self.get_clock().now().nanoseconds)
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = "payload_lab_frd"
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, pos)
            out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, quat)
            self.pose_out.publish(out)
            self.accepted += 1
            self.last_good = time.monotonic()
            self.last_error = ""
        except ValueError as exc:
            self.rejected += 1
            self.last_error = str(exc)

    def report(self):
        age = None if self.last_good is None else time.monotonic() - self.last_good
        status = dict(mode="preview", forwarded_to_px4=False,
                      accepted=self.accepted, rejected=self.rejected,
                      age_s=age, stale=age is None or age > self.config["max_local_stamp_age_s"],
                      last_error=self.last_error, flight_approval=False,
                      server_timing_verified=False, mounting_confirmed=False,
                      lever_arm_measured=False)
        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.status_out.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    options, ros_args = ap.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = MocapPreview(options.config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
