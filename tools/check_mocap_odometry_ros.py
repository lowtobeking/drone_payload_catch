#!/usr/bin/env python3
"""Synthetic end-to-end ROS tests; never discovers or targets a real FC."""
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

if os.environ.get("ROS_DOMAIN_ID") != "192" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("requires localhost-only isolated ROS domain 192")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleLandDetected, TimesyncStatus, EstimatorStatusFlags
from payload_catch.mocap_odometry import load_config
from payload_catch.mocap_odometry_node import MocapOdometry

rclpy.init(args=[])
driver = Node("mocap_odometry_synthetic_driver", start_parameter_services=False)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
raw_pub = driver.create_publisher(String, "/payload/mocap/a/raw_sample", qos)
status_pub = driver.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", qos)
land_pub = driver.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", qos)
sync_pub = driver.create_publisher(TimesyncStatus, "/fmu/out/timesync_status", qos)
flags_pub = driver.create_publisher(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", qos)
previews, fc_inputs = [], []
subscriptions = [
    driver.create_subscription(VehicleOdometry, "/payload/mocap/a/odometry_preview", previews.append, qos),
    driver.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", fc_inputs.append, qos)]
node = None
fixed_proof_path = None
fixed_conflict = False
fixed_height_mode = False
fixed_height_reference = 3
last_raw = 0.
last_state = 0.


def spin(seconds, *, raw=False, armed=False, dynamic=False):
    global last_raw, last_state
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        mono = time.monotonic()
        wall = time.time_ns() // 1000
        if raw and mono - last_raw >= .012:
            msg = String()
            msg.data = json.dumps(dict(schema=1, tracker="planeA", sensor=0, frame="world",
                server_stamp_us=wall + 43_200_000_000 - 5000,
                receive_wall_us=wall, receive_monotonic_us=time.monotonic_ns() // 1000,
                position=[1., 2., 3. + (mono % 1)*1e-6 if dynamic else 3.], quaternion_xyzw=[0., 0., 1., 0.]))
            raw_pub.publish(msg)
            last_raw = mono
        if mono - last_state >= .1:
            status = VehicleStatus()
            status.system_id, status.component_id = 1, 1
            status.arming_state = 2 if armed else 1
            status_pub.publish(status)
            land = VehicleLandDetected()
            land.landed = True
            land_pub.publish(land)
            sync = TimesyncStatus()
            sync.timestamp = wall
            sync.source_protocol = 2
            sync.remote_timestamp = wall - 100
            sync.estimated_offset = -1_700_000_000_000_000
            sync.round_trip_time = 500
            sync_pub.publish(sync)
            flags = EstimatorStatusFlags()
            flags.cs_mag_hdg = fixed_conflict
            flags_pub.publish(flags)
            if fixed_proof_path is not None:
                proof = dict(schema=1,jetson_serial="1612425621719",jetson_boot_id="synthetic-only",
                    fc_uid=3544984969635313204,readback_monotonic_us=time.monotonic_ns()//1000,
                    parameters={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1},fc_restart_verified=True)
                if fixed_height_mode:
                    proof["parameters"].update(EKF2_EV_CTRL=11,EKF2_HGT_REF=fixed_height_reference)
                target = fixed_proof_path.with_suffix(".tmp")
                target.write_text(json.dumps(proof),encoding="utf-8")
                target.replace(fixed_proof_path)
            last_state = mono
        if node is not None:
            rclpy.spin_once(node, timeout_sec=.002)
        rclpy.spin_once(driver, timeout_sec=.002)


def destroy_bridge():
    global node
    if node is not None:
        node.destroy_node()
        node = None
    spin(.4)


result = {"synthetic": True, "domain": 192, "flight_approval": False}
try:
    node = MocapOdometry(ROOT / "config/mocap_a.bridge.yaml")
    spin(.7, raw=True)
    assert previews and node.builder.accepted > 0
    assert not fc_inputs and not node.get_publishers_info_by_topic("/fmu/in/vehicle_visual_odometry")
    assert not node.builder.sample_time_verified
    result["default_preview_without_fc_publisher"] = True
    # Verify that a missing confirmation cannot be bypassed by a CLI output flag.
    try:
        MocapOdometry(ROOT / "config/mocap_a.bridge.yaml", send_to_px4=True, isolated_test=True)
    except ValueError:
        result["unconfirmed_config_refused"] = True
    else:
        raise AssertionError("unconfirmed output was allowed")
    destroy_bridge()

    receive_cfg = ROOT / "config/mocap_a.receive_ground.yaml"
    try:
        MocapOdometry(receive_cfg, send_to_px4=True, isolated_test=True)
    except ValueError:
        result["receive_mode_requires_explicit_cli_ack"] = True
    else:
        raise AssertionError("receive ground mode did not require explicit acknowledgement")
    node = MocapOdometry(receive_cfg, send_to_px4=True, isolated_test=True, ground_receive_time=True)
    spin(1.2, raw=True, dynamic=True)
    assert node.published > 5 and not node.builder.sample_time_verified
    assert 0 <= fc_inputs[-1].timestamp - fc_inputs[-1].timestamp_sample < 50000
    result["receive_mode_native_stamp_forwarded"] = True
    spin(.2, raw=True, armed=True, dynamic=True)
    stopped = node.published
    spin(.3, raw=True, dynamic=True)
    assert node.ground.fault and node.published == stopped
    result["receive_mode_arming_latches_output_off"] = True
    destroy_bridge()

    candidate, _ = load_config(receive_cfg)
    a = .4
    c, t = math.cos(a), math.sin(a)
    base = copy.deepcopy(candidate["transform"]["world_to_local"])
    candidate["transform"]["world_to_local"] = [[-t,c,0.],[c,t,0.],[0.,0.,-1.]]
    candidate["output_pose_frame"] = "px4_heading_aligned_ned"
    candidate["heading_reference"] = dict(method="px4_magnetic_snapshot",north_verified=False,
        yaw_offset_rad=a,std_deg=.1,span_deg=.2,collected_wall_us=time.time_ns()//1000,
        boot_id="a006295d-9ab4-41c6-a277-f0a2455a86a8",base_world_to_local=base)
    with tempfile.TemporaryDirectory(dir=ROOT/"report/mocap_fusion_20261023") as tmp:
        path = Path(tmp)/"SYNTHETIC_HEADING_ONLY.yaml"
        path.write_text(yaml.safe_dump(candidate),encoding="utf-8")
        node = MocapOdometry(path,send_to_px4=True,isolated_test=True,ground_receive_time=True)
        spin(1.2,raw=True,dynamic=True)
        assert node.published > 5 and fc_inputs[-1].pose_frame == 1
        out = fc_inputs[-1]
        assert abs(out.position[0]-(2*c-t))<1e-6 and abs(out.position[1]-(2*t+c))<1e-6
        assert abs(out.q[0]-math.cos(a/2))<1e-6 and abs(out.q[3]-math.sin(a/2))<1e-6
        assert node.cfg["heading_reference"]["north_verified"] is False
        result["snapshot_rotates_position_and_orientation"] = True
        spin(.2,raw=True,armed=True,dynamic=True)
        stopped = node.published
        spin(.3,raw=True,dynamic=True)
        assert node.ground.fault and node.published == stopped
        result["heading_candidate_arming_latches_output_off"] = True
        destroy_bridge()
        candidate["heading_reference"]["collected_wall_us"] = time.time_ns()//1000-601_000_000
        path.write_text(yaml.safe_dump(candidate),encoding="utf-8")
        try:
            MocapOdometry(path,send_to_px4=True,isolated_test=True,ground_receive_time=True)
        except ValueError as exc:
            assert "stale" in str(exc)
            result["expired_heading_snapshot_refused"] = True
        else:
            raise AssertionError("expired heading snapshot accepted")
        assert not driver.get_publishers_info_by_topic("/fmu/in/vehicle_visual_odometry")

    with tempfile.TemporaryDirectory(dir=ROOT/"report/fixed_reference_20261023") as tmp:
        proof_path = Path(tmp)/"readback.json"
        fixed_proof_path = proof_path
        node = MocapOdometry(ROOT/"config/mocap_a.fixed_ground.yaml",send_to_px4=True,
            isolated_test=True,ground_receive_time=True,fixed_parameter_proof=proof_path)
        spin(1.2,raw=True,dynamic=True)
        assert node.published>5 and fc_inputs[-1].pose_frame==1
        assert list(fc_inputs[-1].position[:2])==[2.,1.]
        result["fixed_reference_fresh_proof_forwarded"] = True
        fixed_conflict = True
        spin(.3,raw=True,dynamic=True)
        stopped=node.published
        fixed_conflict = False
        spin(.3,raw=True,dynamic=True)
        assert node.fixed_fault and node.published==stopped
        result["fixed_reference_magnetic_conflict_latches_off"] = True
        destroy_bridge()
        node = MocapOdometry(ROOT/"config/mocap_a.fixed_ground.yaml",send_to_px4=True,
            isolated_test=True,ground_receive_time=True,fixed_parameter_proof=proof_path)
        spin(1.,raw=True,dynamic=True)
        assert node.published>5
        fixed_proof_path = None
        spin(2.3,raw=True,dynamic=True)
        stopped=node.published
        assert node.fixed_fault
        spin(.2,raw=True,dynamic=True)
        assert node.published==stopped
        result["fixed_reference_stale_parameter_proof_latches_off"] = True
        destroy_bridge()

    with tempfile.TemporaryDirectory(dir=ROOT/"report/mocap_z_20261023") as tmp:
        proof_path=Path(tmp)/"xyz_readback.json"
        fixed_proof_path=proof_path
        fixed_height_mode=True
        node=MocapOdometry(ROOT/"config/mocap_a.xyz_ground.yaml",send_to_px4=True,
            isolated_test=True,ground_receive_time=True,fixed_parameter_proof=proof_path)
        spin(1.2,raw=True,dynamic=True)
        assert node.published>5 and abs(fc_inputs[-1].position[2]+3)<.00001
        result["xyz_height_proof_and_negative_z_forwarded"]=True
        fixed_height_reference=1
        spin(.3,raw=True,dynamic=True)
        stopped=node.published
        assert node.fixed_fault
        fixed_height_reference=3
        spin(.3,raw=True,dynamic=True)
        assert node.published==stopped
        result["xyz_wrong_height_reference_latches_off"]=True
        destroy_bridge()
        fixed_height_mode=False
        fixed_proof_path=None

    cfg, _ = load_config(ROOT / "config/mocap_a.bridge.yaml")
    for key in ("enabled", "mounting_confirmed", "lever_arm_measured",
                "server_timing_verified", "tracking_loss_verified"):
        cfg[key] = True
    cfg["source_to_local_offset_us"] = -43_200_000_000
    cfg["source_offset_uncertainty_us"] = 1000
    with tempfile.TemporaryDirectory(dir=ROOT / "report/mocap_fusion_20261023") as tmp:
        path = Path(tmp) / "SYNTHETIC_ONLY.yaml"
        path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
        node = MocapOdometry(path, send_to_px4=True, isolated_test=True)
        spin(1.2, raw=True)
        assert node.published > 5 and fc_inputs
        out = fc_inputs[-1]
        assert out.pose_frame == VehicleOdometry.POSE_FRAME_FRD
        assert list(out.position) == [2., 1., -3.]
        assert abs(abs(out.q[0]) - 1.) < 1e-6
        assert out.timestamp >= out.timestamp_sample
        assert 4000 <= out.timestamp - out.timestamp_sample < 100000
        assert all(math.isnan(x) for x in out.velocity)
        result["synthetic_live_message_format"] = True
        before = node.published
        spin(.2, raw=True, armed=True)
        stopped = node.published
        spin(.35, raw=True)
        assert node.ground.fault and node.published == stopped
        result["arming_latches_output_off"] = True
        destroy_bridge()

        node = MocapOdometry(path, send_to_px4=True, isolated_test=True)
        spin(1.2, raw=True)
        assert node.published > 5
        spin(.3, raw=False)
        stopped = node.published
        spin(.35, raw=True)
        assert node.builder.fault and node.published == stopped
        result["dropout_does_not_auto_resume"] = True
        destroy_bridge()

        cfg["source_to_local_offset_us"] = 0
        path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
        node = MocapOdometry(path, send_to_px4=True, isolated_test=True)
        spin(.8, raw=True)
        assert node.published == 0 and node.rejected > 0
        result["twelve_hour_wrong_offset_blocks_output"] = True
        for suffix in ("vehicle_command", "offboard_control_mode", "trajectory_setpoint",
                       "actuator_motors", "actuator_servos"):
            assert not node.get_publishers_info_by_topic("/fmu/in/" + suffix), suffix
        result["flight_and_actuator_publishers"] = 0
    result["passed"] = True
    (ROOT / "report/mocap_z_20261023/ros_validation.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))
finally:
    destroy_bridge()
    driver.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
