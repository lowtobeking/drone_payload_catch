from pathlib import Path
import ast
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/"tools/check_mocap_odometry_ros.py"
s=p.read_text(encoding="utf-8")
s=s.replace("TimesyncStatus\n","TimesyncStatus, EstimatorStatusFlags\n",1)
s=s.replace('last_raw = 0.','fixed_proof_path = None\nfixed_conflict = False\nlast_raw = 0.',1)
s=s.replace('sync_pub = driver.create_publisher(TimesyncStatus, "/fmu/out/timesync_status", qos)',
'''sync_pub = driver.create_publisher(TimesyncStatus, "/fmu/out/timesync_status", qos)
flags_pub = driver.create_publisher(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", qos)''',1)
s=s.replace('            sync_pub.publish(sync)\n','''            sync_pub.publish(sync)
            flags = EstimatorStatusFlags()
            flags.cs_mag_hdg = fixed_conflict
            flags_pub.publish(flags)
            if fixed_proof_path is not None:
                proof = dict(schema=1,jetson_serial="1612425621719",jetson_boot_id="synthetic-only",
                    fc_uid=3544984969635313204,readback_monotonic_us=time.monotonic_ns()//1000,
                    parameters={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1},fc_restart_verified=True)
                target = fixed_proof_path.with_suffix(".tmp")
                target.write_text(json.dumps(proof),encoding="utf-8")
                target.replace(fixed_proof_path)
''',1)
needle='    cfg, _ = load_config(ROOT / "config/mocap_a.bridge.yaml")'
addition='''    with tempfile.TemporaryDirectory(dir=ROOT/"report/fixed_reference_20261023") as tmp:
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

'''
assert needle in s
s=s.replace(needle,addition+needle,1)
ast.parse(s)
p.write_text(s,encoding="utf-8")
