"""PX4 external-vision message preparation and ground gates, without ROS or IO.

Timestamp values here are LOCAL companion Unix microseconds. PX4's enabled
uXRCE-DDS time translation converts both timestamps to FC time on receive.
Never subtract estimated_offset again in application code.
"""
from pathlib import Path
import math

import numpy as np

import yaml

from .mocap_transform import Transform

CONFIRMATIONS = ("mounting_confirmed", "lever_arm_measured",
                 "server_timing_verified", "tracking_loss_verified")


def number(value, name, lo, hi):
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(name + " outside allowed range")
    return value


def integer(value, name, lo, hi):
    if type(value) is not int or not lo <= value <= hi:
        raise ValueError(name + " must be an integer in range")
    return value


def live_issues(cfg, *, ground_receive_time=False):
    if cfg.get("timestamp_mode", "verified_source") == "receive_time_ground":
        issues = []
        if cfg["enabled"] is not True:
            issues.append("enabled must explicitly be true")
        if not ground_receive_time:
            issues.append("explicit --ground-receive-time acknowledgement required")
        if cfg.get("ground_receive_time_acknowledged") is not True:
            issues.append("receive-time / unmeasured geometry ground-test limitations not acknowledged")
        return issues
    if ground_receive_time:
        return ["--ground-receive-time requires receive_time_ground configuration"]
    issues = [name + " not confirmed" for name in CONFIRMATIONS if cfg[name] is not True]
    if cfg["enabled"] is not True:
        issues.append("enabled must explicitly be true")
    if cfg["source_to_local_offset_us"] is None or cfg["source_offset_uncertainty_us"] is None:
        issues.append("measured source time offset and uncertainty required")
    return issues


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or type(cfg.get("schema")) is not int or cfg["schema"] != 1:
        raise ValueError("unsupported bridge schema")
    if cfg.get("scope") != "ground_only":
        raise ValueError("this bridge currently supports ground commissioning only")
    if cfg.get("input_topic") != "/payload/mocap/a/raw_sample":
        raise ValueError("raw source envelope is required, not preview PoseStamped")
    if cfg.get("output_topic") != "/fmu/in/vehicle_visual_odometry":
        raise ValueError("only A external vision input is supported")
    if (cfg.get("tracker"), cfg.get("sensor"), cfg.get("input_frame")) != ("planeA", 0, "world"):
        raise ValueError("explicit planeA sensor 0 / world required")
    integer(cfg.get("sensor"), "sensor", 0, 0)
    for name in ("enabled",) + CONFIRMATIONS:
        if type(cfg.get(name)) is not bool:
            raise ValueError(name + " must be boolean")
    for name, lo, hi in (
        ("max_sample_age_s", .02, .3), ("max_receive_age_s", .005, .1),
        ("max_source_gap_s", .02, .3), ("max_clock_step_s", .001, .05),
        ("max_time_sync_age_s", 1., 3.), ("max_time_sync_rtt_us", 100, 20000),
        ("max_output_hz", 20., 100.), ("position_variance_m2", .0001, 1.),
        ("orientation_variance_rad2", .0001, .25)):
        number(cfg.get(name), name, lo, hi)
    integer(cfg.get("expected_system_id"), "expected_system_id", 1, 1)
    integer(cfg.get("expected_component_id"), "expected_component_id", 1, 1)
    integer(cfg.get("reset_counter"), "reset_counter", 0, 255)
    offset, uncertainty = cfg.get("source_to_local_offset_us"), cfg.get("source_offset_uncertainty_us")
    if offset is not None:
        integer(offset, "source_to_local_offset_us", -10**16, 10**16)
    if uncertainty is not None:
        integer(uncertainty, "source_offset_uncertainty_us", 0, 10000)
    mode = cfg.get("timestamp_mode", "verified_source")
    if mode not in ("verified_source", "receive_time_ground"):
        raise ValueError("unknown timestamp_mode")
    if mode == "receive_time_ground":
        if type(cfg.get("ground_receive_time_acknowledged")) is not bool:
            raise ValueError("ground receive acknowledgement must be boolean")
        number(cfg.get("max_relative_time_drift_s"), "max_relative_time_drift_s", .02, .1)
        number(cfg.get("max_identical_pose_s"), "max_identical_pose_s", .2, 1.)
    frame = cfg.get("output_pose_frame", "local_frd")
    if frame not in ("local_frd", "px4_heading_aligned_ned", "fixed_lab_navigation"):
        raise ValueError("unsupported output_pose_frame")
    if frame == "px4_heading_aligned_ned":
        ref = cfg.get("heading_reference")
        if mode != "receive_time_ground" or not isinstance(ref, dict):
            raise ValueError("snapshot heading alignment requires explicit receive-time ground mode")
        if ref.get("method") != "px4_magnetic_snapshot" or ref.get("north_verified") is not False:
            raise ValueError("explicit unverified magnetic snapshot provenance required")
        number(ref.get("yaw_offset_rad"), "heading offset", -math.pi, math.pi)
        number(ref.get("std_deg"), "heading snapshot std", 0., 1.)
        number(ref.get("span_deg"), "heading snapshot span", 0., 3.)
        integer(ref.get("collected_wall_us"), "heading snapshot time", 1, 2**63-1)
        if not isinstance(ref.get("boot_id"), str) or len(ref["boot_id"]) != 36:
            raise ValueError("heading snapshot boot id required")
        base = np.asarray(ref.get("base_world_to_local"), dtype=float)
        a = ref["yaw_offset_rad"]
        c, t = math.cos(a), math.sin(a)
        rz = np.asarray([[c, -t, 0.], [t, c, 0.], [0., 0., 1.]])
        if base.shape != (3, 3) or not np.allclose(
                np.asarray(cfg["transform"]["world_to_local"]), rz @ base, atol=1e-8):
            raise ValueError("position and attitude must use the same measured heading rotation")
    if frame == "fixed_lab_navigation":
        ref = cfg.get("fixed_reference")
        if mode != "receive_time_ground" or not isinstance(ref, dict):
            raise ValueError("fixed laboratory reference requires explicit receive-time ground mode")
        if (ref.get("id") != "aerolab_world_Y_X_minusZ_v1"
                or ref.get("convention") != "indoor_navigation_x_reference_y_right_z_down"
                or ref.get("geographic_north_verified") is not False
                or ref.get("magnetic_alignment") != "disabled"):
            raise ValueError("explicit fixed indoor reference contract required")
        if not np.allclose(cfg["transform"]["world_to_local"],
                           [[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]],atol=1e-12,rtol=0):
            raise ValueError("fixed laboratory axes must not follow magnetic or current body yaw")
        if not np.allclose(cfg["transform"]["shared_origin_world_m"],[0.,0.,0.],atol=0,rtol=0):
            raise ValueError("fixed shared origin cannot be reset at each pose")
    height_mode=cfg.get("height_fusion","not_requested")
    if height_mode not in ("not_requested","external_vision_reference"):
        raise ValueError("unknown height fusion contract")
    if height_mode=="external_vision_reference":
        if frame!="fixed_lab_navigation" or mode!="receive_time_ground":
            raise ValueError("external height reference requires fixed disarmed ground mode")
        if cfg.get("height_origin")!="fixed_laboratory_marker_origin_not_agl":
            raise ValueError("explicit marker/world height origin required")
    return cfg, Transform(**cfg["transform"])


class OdometryBuilder:
    """Single-stream lifetime. Source dropout/clock regressions latch until restart."""
    def __init__(self, cfg, transform):
        self.cfg, self.transform = cfg, transform
        self.fault = ""
        self.last_source = self.last_receive = self.last_good_mono = None
        self.previous_clock_pair = None
        self.last_publish_mono = None
        self.accepted = self.rejected = 0
        self.receive_time_ground = cfg.get("timestamp_mode") == "receive_time_ground"
        self.relative_pair = None
        self.last_pose = self.pose_changed_mono = None
        self.sample_time_verified = (not self.receive_time_ground
                                     and cfg["server_timing_verified"] is True
                                     and cfg["source_to_local_offset_us"] is not None
                                     and cfg["source_offset_uncertainty_us"] is not None)

    def fail(self, reason):
        self.fault = reason
        raise ValueError(reason)

    def tick(self, now_wall_us, now_mono_us):
        integer(now_wall_us, "local wall time", 1, 2**63 - 1)
        integer(now_mono_us, "local monotonic time", 1, 2**63 - 1)
        if self.previous_clock_pair is not None:
            wall, mono = self.previous_clock_pair
            if (now_mono_us < mono or
                    abs((now_wall_us - wall) - (now_mono_us - mono))
                    > self.cfg["max_clock_step_s"] * 1e6):
                self.fault = "local clock stepped; explicit restart required"
        self.previous_clock_pair = now_wall_us, now_mono_us
        if (self.last_good_mono is not None and
                now_mono_us - self.last_good_mono > self.cfg["max_source_gap_s"] * 1e6):
            self.fault = self.fault or "source dropout; explicit restart required"
        if self.fault:
            raise ValueError(self.fault)

    def accept(self, data, now_wall_us, now_mono_us):
        self.tick(now_wall_us, now_mono_us)
        if (not isinstance(data, dict) or type(data.get("schema")) is not int or data["schema"] != 1
                or data.get("tracker") != self.cfg["tracker"]
                or type(data.get("sensor")) is not int or data["sensor"] != 0
                or data.get("frame") != self.cfg["input_frame"]):
            raise ValueError("wrong source envelope")
        source = integer(data.get("server_stamp_us"), "source stamp", 1, 2**63 - 1)
        recv_wall = integer(data.get("receive_wall_us"), "receive wall time", 1, 2**63 - 1)
        recv_mono = integer(data.get("receive_monotonic_us"), "receive monotonic time", 1, 2**63 - 1)
        if self.last_source is not None:
            if source < self.last_source:
                self.fail("source clock regressed; explicit restart required")
            if source == self.last_source:
                raise ValueError("duplicate source timestamp")
            if source - self.last_source > self.cfg["max_source_gap_s"] * 1e6:
                self.fail("source timestamp gap; explicit restart required")
        if self.last_receive is not None and recv_mono < self.last_receive:
            self.fail("native receive clock regressed")
        receive_age = now_mono_us - recv_mono
        if not 0 <= receive_age <= self.cfg["max_receive_age_s"] * 1e6:
            raise ValueError("queued/stale or future native receive time")
        if abs((now_wall_us - recv_wall) - receive_age) > self.cfg["max_clock_step_s"] * 1e6:
            self.fail("native and ROS local clocks disagree")
        if self.receive_time_ground:
            if self.relative_pair is None:
                self.relative_pair = source, recv_mono
            else:
                base_source, base_receive = self.relative_pair
                if abs((source - base_source) - (recv_mono - base_receive)) > self.cfg["max_relative_time_drift_s"] * 1e6:
                    self.fail("source/receive relative clock drift or network backlog; restart required")
        sample = (source + self.cfg["source_to_local_offset_us"]
                  if self.sample_time_verified else recv_wall)
        # Receive time is a bounded ground-test approximation, never verified exposure time.
        if not 0 < sample <= now_wall_us:
            raise ValueError("mapped sample time is missing or in future")
        uncertainty = self.cfg["source_offset_uncertainty_us"] or 0
        if now_wall_us - sample + uncertainty > self.cfg["max_sample_age_s"] * 1e6:
            raise ValueError("mapped source sample is stale")
        position, xyzw = self.transform.convert(data.get("position"), data.get("quaternion_xyzw"))
        if any(abs(v) > 10000 for v in position):
            raise ValueError("position outside supported range")
        if self.receive_time_ground:
            pose = tuple(position) + tuple(xyzw)
            if self.last_pose != pose:
                self.last_pose, self.pose_changed_mono = pose, recv_mono
            elif recv_mono - self.pose_changed_mono > self.cfg["max_identical_pose_s"] * 1e6:
                self.fail("exactly repeated pose stream; restart required")
        self.last_source, self.last_receive = source, recv_mono
        self.last_good_mono = recv_mono
        self.accepted += 1
        if (self.last_publish_mono is not None and
                now_mono_us - self.last_publish_mono < 1e6 / self.cfg["max_output_hz"]):
            return None
        self.last_publish_mono = now_mono_us
        nan3 = [math.nan] * 3
        return dict(timestamp=now_wall_us, timestamp_sample=sample,
                    pose_frame=(1 if self.cfg.get("output_pose_frame") in ("px4_heading_aligned_ned", "fixed_lab_navigation") else 2),
                    position=list(map(float, position)),
                    q=[float(xyzw[3]), *map(float, xyzw[:3])],
                    velocity_frame=0, velocity=nan3[:], angular_velocity=nan3[:],
                    position_variance=[float(self.cfg["position_variance_m2"])] * 3,
                    orientation_variance=[float(self.cfg["orientation_variance_rad2"])] * 3,
                    velocity_variance=nan3[:], reset_counter=self.cfg["reset_counter"],
                    quality=0)


class GroundGate:
    """For commissioning only: disarmed+landed+fresh DDS time synchronization."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.status = self.land = self.sync = None
        self.sync_count = 0
        self.sync_offset = None
        self.last_sync_remote = None
        self.fault = ""
        self.was_ready = False

    def record_status(self, msg, now_mono_us):
        self.status = (msg, now_mono_us)

    def record_land(self, msg, now_mono_us):
        self.land = (msg, now_mono_us)

    def record_sync(self, msg, now_wall_us, now_mono_us):
        if self.last_sync_remote is not None and msg.remote_timestamp <= self.last_sync_remote:
            return  # A replay must not refresh time synchronization health.
        self.last_sync_remote = msg.remote_timestamp
        valid = (msg.source_protocol == 2
                 and 0 <= now_wall_us - msg.remote_timestamp <= 1_000_000
                 and 0 < msg.round_trip_time <= self.cfg["max_time_sync_rtt_us"])
        if (valid and self.sync_offset is not None
                and abs(msg.estimated_offset - self.sync_offset) > 5000):
            valid = False
        self.sync_count = self.sync_count + 1 if valid else 0
        self.sync_offset = msg.estimated_offset if valid else None
        self.sync = (valid, now_mono_us)

    def check(self, now_mono_us):
        reasons = []
        if self.fault:
            return [self.fault]
        if self.status is None or now_mono_us - self.status[1] > 1_500_000:
            reasons.append("A vehicle status missing/stale")
        else:
            s = self.status[0]
            if (s.system_id, s.component_id) != (1, 1):
                reasons.append("wrong FC identity")
            if s.arming_state != 1:
                reasons.append("ground commissioning requires disarmed")
        if self.land is None or now_mono_us - self.land[1] > 1_500_000 or not self.land[0].landed:
            reasons.append("landed confirmation missing/stale/false")
        if (self.sync is None or not self.sync[0] or self.sync_count < 3
                or now_mono_us - self.sync[1] > self.cfg["max_time_sync_age_s"] * 1e6):
            reasons.append("DDS clock synchronization not verified/fresh")
        if self.was_ready and reasons:
            self.fault = "; ".join(reasons) + "; restart required"
        if not reasons:
            self.was_ready = True
        return reasons


def fixed_parameter_issues(policy, now_mono_us, boot_id, *, height_fusion=False):
    if not isinstance(policy, dict) or policy.get("schema") != 1:
        return ["fixed reference parameter proof missing"]
    if (policy.get("jetson_serial") != "1612425621719" or policy.get("jetson_boot_id") != boot_id
            or policy.get("fc_uid") != 3544984969635313204):
        return ["fixed reference parameter proof identity mismatch"]
    stamp = policy.get("readback_monotonic_us")
    if type(stamp) is not int or not 0 <= now_mono_us-stamp <= 2_000_000:
        return ["fixed reference parameter readback stale/future"]
    required={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1}
    if height_fusion:
        required.update(EKF2_EV_CTRL=11,EKF2_HGT_REF=3)
    if policy.get("parameters") != required:
        return ["fixed reference parameters do not match explicit XY/yaw or XYZ/yaw height contract"]
    if policy.get("fc_restart_verified") is not True:
        return ["MAG_TYPE restart application not verified"]
    return []
