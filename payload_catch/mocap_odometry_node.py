"""A-only bounded ground commissioning: raw VRPN -> PX4 VehicleOdometry.

Default is diagnostic output ONLY. --send-to-px4 requires either confirmed
source timing/geometry or the explicit receive-time ground-test acknowledgement,
plus A hardware identity and continuously disarmed/landed telemetry.
No parameter, flight-command, setpoint, motor or PWM interfaces.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleLandDetected, TimesyncStatus, EstimatorStatusFlags
from std_msgs.msg import String

from .mocap_odometry import GroundGate, OdometryBuilder, live_issues, load_config, fixed_parameter_issues


def clock_pair(node):
    return node.get_clock().now().nanoseconds // 1000, time.monotonic_ns() // 1000


class MocapOdometry(Node):
    def __init__(self, config, *, send_to_px4=False, isolated_test=False, ground_receive_time=False, flight_receive_time=False, fixed_parameter_proof=None):
        cfg, transform = load_config(config)
        if isolated_test and (os.environ.get("ROS_DOMAIN_ID") != "192"
                              or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
                              or Path("/proc/device-tree/serial-number").exists()):
            raise ValueError("synthetic forwarding tests require non-Jetson localhost domain 192")
        if send_to_px4:
            issues = live_issues(cfg, ground_receive_time=ground_receive_time, flight_receive_time=flight_receive_time)
            if issues:
                raise ValueError("PX4 output refused: " + "; ".join(issues))
            if not isolated_test:
                serial = Path("/proc/device-tree/serial-number").read_text().strip("\0\n")
                if serial != "1612425621719":
                    raise ValueError("not verified A hardware")
        if send_to_px4 and cfg.get("output_pose_frame") == "px4_heading_aligned_ned":
            ref = cfg["heading_reference"]
            if not 0 <= time.time_ns() // 1000 - ref["collected_wall_us"] <= 600_000_000:
                raise ValueError("heading snapshot stale/future; recollect explicitly")
            if not isolated_test and Path("/proc/sys/kernel/random/boot_id").read_text().strip() != ref["boot_id"]:
                raise ValueError("heading snapshot belongs to another Jetson boot")
        super().__init__("a_mocap_odometry", start_parameter_services=False)
        self.flight_mode = cfg["scope"] == "a_solo_flight"
        self.flight_started_mono = time.monotonic_ns() // 1000
        self.ev_seen = False
        self.fixed_mode = cfg.get("output_pose_frame") == "fixed_lab_navigation"
        self.fixed_proof_path = Path(fixed_parameter_proof) if fixed_parameter_proof else None
        self.fixed_boot = (Path("/proc/sys/kernel/random/boot_id").read_text().strip()
                           if not isolated_test else "synthetic-only")
        self.fixed_status = None
        self.fixed_fault = ""
        self.fixed_ready = False
        self.cfg = cfg
        self.send_to_px4 = send_to_px4
        self.ground_receive_time = ground_receive_time
        self.flight_receive_time = flight_receive_time
        self.builder = OdometryBuilder(cfg, transform)
        self.ground = GroundGate(cfg)
        self.published = self.rejected = 0
        self.reason = "waiting for planeA"
        self.last_output_mono = None
        self.external_fault = ""
        if self.get_parameter("use_sim_time").value:
            self.destroy_node()
            raise ValueError("hardware timestamps require real ROS time")
        if (VehicleOdometry.MESSAGE_VERSION != 0 or VehicleOdometry.POSE_FRAME_FRD != 2
                or VehicleOdometry.VELOCITY_FRAME_UNKNOWN != 0):
            self.destroy_node()
            raise ValueError("unsupported px4_msgs VehicleOdometry schema")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.preview = self.create_publisher(VehicleOdometry, "/payload/mocap/a/odometry_preview", qos)
        self.status_out = self.create_publisher(String, "/payload/mocap/a/bridge_status", 5)
        self.px4 = (self.create_publisher(VehicleOdometry, cfg["output_topic"], qos)
                    if send_to_px4 else None)
        self.raw_sub = self.create_subscription(String, cfg["input_topic"], self.receive, qos)
        self.status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1",
            lambda msg: self.ground.record_status(msg, time.monotonic_ns() // 1000), qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            lambda msg: self.ground.record_land(msg, time.monotonic_ns() // 1000), qos)
        self.sync_sub = self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status",
            lambda msg: self.ground.record_sync(msg, *clock_pair(self)), qos)
        if self.fixed_mode:
            self.flags_sub = self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags",
                lambda msg: setattr(self, "fixed_status", (msg, time.monotonic_ns()//1000)), qos)
        self.create_timer(.02, self.watch)
        self.create_timer(.5, self.report)
        self.get_logger().warn(("A-ONLY FLIGHT BRIDGE" if self.flight_mode else "GROUND ONLY")
            + ": " + ("explicit PX4 output enabled" if send_to_px4 else "diagnostic; no PX4 publisher"))

    def fixed_issues(self, mono):
        if not self.fixed_mode:
            return []
        if self.fixed_fault:
            return [self.fixed_fault]
        issues = []
        if not self.flight_mode:
            try:
                policy = json.loads(self.fixed_proof_path.read_text(encoding="utf-8")) if self.fixed_proof_path else None
            except (OSError, ValueError):
                policy = None
            # The ground guardian renews this proof while PX4 parameters are leased.
            proof_mono = time.monotonic_ns() // 1000
            issues = fixed_parameter_issues(policy, proof_mono, self.fixed_boot,
                height_fusion=self.cfg.get("height_fusion")=="external_vision_reference")
        if self.fixed_status is None or mono-self.fixed_status[1]>1_500_000:
            issues.append("fixed reference estimator flags missing/stale")
        else:
            flags = self.fixed_status[0]
            if flags.cs_mag_hdg or flags.cs_mag_3d or flags.cs_gnss_pos or flags.cs_gnss_vel or flags.cs_gnss_yaw:
                issues.append("fixed indoor reference conflicts with magnetic/geographic aiding")
            if self.flight_mode:
                ev_ready = flags.cs_ev_pos and flags.cs_ev_yaw and flags.cs_ev_hgt
                if ev_ready:
                    self.ev_seen = True
                elif self.ev_seen or mono - self.flight_started_mono > 10_000_000:
                    issues.append("external vision XYZ/yaw fusion missing")
        if self.fixed_ready and issues:
            self.fixed_fault = "; ".join(issues)+"; restart required"
        if not issues:
            self.fixed_ready = True
        return issues

    def watch(self):
        wall, mono = clock_pair(self)
        try:
            self.builder.tick(wall, mono)
        except ValueError as exc:
            self.reason = str(exc)
        if self.send_to_px4:
            reasons = self.ground.check(mono) + self.fixed_issues(mono)
            if reasons:
                self.reason = "; ".join(reasons)
            if len(self.get_publishers_info_by_topic(self.cfg["input_topic"])) > 1:
                self.external_fault = "multiple raw source publishers; restart required"
            infos = self.get_publishers_info_by_topic(self.cfg["output_topic"])
            if (len(infos) > 1 or any(i.node_name != self.get_name()
                    or i.node_namespace != self.get_namespace() for i in infos)):
                self.external_fault = "competing PX4 odometry publisher; restart required"

    def receive(self, msg):
        try:
            if len(msg.data) > 4096:
                raise ValueError("raw envelope too large")
            data = json.loads(msg.data)
            wall, mono = clock_pair(self)
            values = self.builder.accept(data, wall, mono)
            if values is None:
                return
            out = VehicleOdometry()
            for key, value in values.items():
                setattr(out, key, value)
            self.preview.publish(out)
            if self.px4 is not None:
                self.watch()
                reasons = self.ground.check(mono) + self.fixed_issues(mono)
                if self.external_fault:
                    reasons.append(self.external_fault)
                if self.builder.fault:
                    reasons.append(self.builder.fault)
                if reasons:
                    self.reason = "; ".join(reasons)
                    return
                self.px4.publish(out)
                self.published += 1
                self.last_output_mono = mono
            self.reason = ""
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self.rejected += 1
            self.builder.rejected += 1
            self.reason = str(exc)

    def report(self):
        _, mono = clock_pair(self)
        reasons = self.ground.check(mono) + self.fixed_issues(mono)
        result = dict(scope=self.cfg["scope"], send_to_px4_requested=self.send_to_px4,
                      accepted=self.builder.accepted, rejected=self.rejected,
                      forwarded=self.published,
                      stale=self.builder.last_good_mono is None or
                      mono - self.builder.last_good_mono > self.cfg["max_source_gap_s"] * 1e6,
                      latched_fault=self.builder.fault or self.ground.fault or self.external_fault or self.fixed_fault,
                      last_error=self.reason, ground_gate_issues=reasons,
                      configuration_issues=live_issues(self.cfg, ground_receive_time=self.ground_receive_time,
                                                        flight_receive_time=self.flight_receive_time),
                      timestamp_mode=self.cfg.get("timestamp_mode", "verified_source"),
                      output_pose_frame=self.cfg.get("output_pose_frame", "local_frd"),
                      heading_reference=self.cfg.get("heading_reference"),
                      fixed_reference=self.cfg.get("fixed_reference"),
                      height_fusion=self.cfg.get("height_fusion","not_requested"),
                      height_origin=self.cfg.get("height_origin"),
                      lever_arm_measured=self.cfg["lever_arm_measured"],
                      limitations=("receive time includes unknown acquisition/network delay; bridge itself gives no flight approval"
                                   if self.builder.receive_time_ground else ""),
                      sample_time_verified=self.builder.sample_time_verified,
                      flight_approval=False, velocity_supplied=False)
        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.status_out.publish(msg)
        self.get_logger().info(msg.data)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--send-to-px4", action="store_true")
    ap.add_argument("--ground-receive-time", action="store_true",
                    help="explicit acknowledgement: unverified sample time/geometry; disarmed ground test only")
    ap.add_argument("--flight-receive-time", action="store_true",
                    help="explicit A-only flight bridge receive-time acknowledgement")
    ap.add_argument("--fixed-parameter-proof", help="fresh same-boot guardian parameter readback file")
    ap.add_argument("--isolated-test", action="store_true")
    ap.add_argument("--seconds", type=float, default=30.)
    ap.add_argument("--continuous-ground", action="store_true", help="continue until stopped, retaining disarmed ground gates; requires --seconds 0")
    ap.add_argument("--continuous-flight", action="store_true", help="A-only flight bridge; requires --seconds 0 and explicit config")
    args = ap.parse_args()
    if args.continuous_ground and (args.seconds != 0 or not args.send_to_px4 or not args.ground_receive_time or args.isolated_test):
        ap.error("continuous ground session requires --seconds 0, --send-to-px4, --ground-receive-time; no isolated test")
    if args.continuous_flight and (args.continuous_ground or args.seconds != 0 or not args.send_to_px4
                                   or not args.flight_receive_time or args.isolated_test):
        ap.error("continuous flight requires --seconds 0, --send-to-px4, --flight-receive-time")
    if not (args.continuous_ground or args.continuous_flight) and not 1 <= args.seconds <= 120:
        ap.error("--seconds must be within 1..120 unless a continuous mode is selected")
    rclpy.init(args=[])
    node = None
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        node = MocapOdometry(args.config, send_to_px4=args.send_to_px4,
                             isolated_test=args.isolated_test, ground_receive_time=args.ground_receive_time,
                             flight_receive_time=args.flight_receive_time, fixed_parameter_proof=args.fixed_parameter_proof)
        end = time.monotonic() + args.seconds
        while not stopping and rclpy.ok() and (args.continuous_ground or args.continuous_flight or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
