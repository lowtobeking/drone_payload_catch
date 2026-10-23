"""Explicit indoor fixed navigation convention, guarded by fresh parameter readbacks."""
from pathlib import Path
import ast
import yaml
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/"payload_catch/mocap_odometry.py"
s=p.read_text(encoding="utf-8")
s=s.replace('("local_frd", "px4_heading_aligned_ned")','("local_frd", "px4_heading_aligned_ned", "fixed_lab_navigation")',1)
needle='    return cfg, Transform(**cfg["transform"])'
addition='''    if frame == "fixed_lab_navigation":
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
'''
assert needle in s
s=s.replace(needle,addition+needle,1)
s=s.replace('self.cfg.get("output_pose_frame") == "px4_heading_aligned_ned"',
            'self.cfg.get("output_pose_frame") in ("px4_heading_aligned_ned", "fixed_lab_navigation")',1)
# Pure policy validator; the node reads an atomic same-boot receipt from the bounded parameter guardian.
s+='''

def fixed_parameter_issues(policy, now_mono_us, boot_id):
    if not isinstance(policy, dict) or policy.get("schema") != 1:
        return ["fixed reference parameter proof missing"]
    if (policy.get("jetson_serial") != "1612425621719" or policy.get("jetson_boot_id") != boot_id
            or policy.get("fc_uid") != 3544984969635313204):
        return ["fixed reference parameter proof identity mismatch"]
    stamp = policy.get("readback_monotonic_us")
    if type(stamp) is not int or not 0 <= now_mono_us-stamp <= 2_000_000:
        return ["fixed reference parameter readback stale/future"]
    if policy.get("parameters") != {"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1}:
        return ["requires MAG_TYPE=5, EV_CTRL=9 and SYNCT=1"]
    if policy.get("fc_restart_verified") is not True:
        return ["MAG_TYPE restart application not verified"]
    return []
'''
ast.parse(s)
p.write_text(s,encoding="utf-8")
p=ROOT/"payload_catch/mocap_odometry_node.py"
s=p.read_text(encoding="utf-8")
s=s.replace("TimesyncStatus\n","TimesyncStatus, EstimatorStatusFlags\n",1)
s=s.replace("live_issues, load_config","live_issues, load_config, fixed_parameter_issues",1)
s=s.replace("ground_receive_time=False):","ground_receive_time=False, fixed_parameter_proof=None):",1)
s=s.replace('        self.cfg = cfg','''        self.fixed_mode = cfg.get("output_pose_frame") == "fixed_lab_navigation"
        self.fixed_proof_path = Path(fixed_parameter_proof) if fixed_parameter_proof else None
        self.fixed_boot = (Path("/proc/sys/kernel/random/boot_id").read_text().strip()
                           if not isolated_test else "synthetic-only")
        self.fixed_status = None
        self.fixed_fault = ""
        self.fixed_ready = False
        self.cfg = cfg''',1)
needle='        self.create_timer(.02, self.watch)'
s=s.replace(needle,'''        if self.fixed_mode:
            self.flags_sub = self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags",
                lambda msg: setattr(self, "fixed_status", (msg, time.monotonic_ns()//1000)), qos)
'''+needle,1)
needle='    def watch(self):'
method='''    def fixed_issues(self, mono):
        if not self.fixed_mode:
            return []
        if self.fixed_fault:
            return [self.fixed_fault]
        try:
            policy = json.loads(self.fixed_proof_path.read_text(encoding="utf-8")) if self.fixed_proof_path else None
        except (OSError, ValueError):
            policy = None
        issues = fixed_parameter_issues(policy, mono, self.fixed_boot)
        if self.fixed_status is None or mono-self.fixed_status[1]>1_500_000:
            issues.append("fixed reference estimator flags missing/stale")
        else:
            flags = self.fixed_status[0]
            if flags.cs_mag_hdg or flags.cs_mag_3d or flags.cs_gnss_pos or flags.cs_gnss_vel or flags.cs_gnss_yaw:
                issues.append("fixed indoor reference conflicts with magnetic/geographic aiding")
        if self.fixed_ready and issues:
            self.fixed_fault = "; ".join(issues)+"; restart required"
        if not issues:
            self.fixed_ready = True
        return issues

'''
s=s.replace(needle,method+needle,1)
s=s.replace('            reasons = self.ground.check(mono)\n','            reasons = self.ground.check(mono) + self.fixed_issues(mono)\n',1)
s=s.replace('                reasons = self.ground.check(mono)\n','                reasons = self.ground.check(mono) + self.fixed_issues(mono)\n',1)
s=s.replace('        reasons = self.ground.check(mono)\n','        reasons = self.ground.check(mono) + self.fixed_issues(mono)\n',1)
s=s.replace('self.ground.fault or self.external_fault,','self.ground.fault or self.external_fault or self.fixed_fault,',1)
s=s.replace('heading_reference=self.cfg.get("heading_reference"),',
            'heading_reference=self.cfg.get("heading_reference"),\n                      fixed_reference=self.cfg.get("fixed_reference"),',1)
s=s.replace('    ap.add_argument("--isolated-test",',
            '    ap.add_argument("--fixed-parameter-proof", help="fresh same-boot guardian parameter readback file")\n    ap.add_argument("--isolated-test",',1)
s=s.replace('ground_receive_time=args.ground_receive_time)',
            'ground_receive_time=args.ground_receive_time,\n                             fixed_parameter_proof=args.fixed_parameter_proof)',1)
ast.parse(s)
p.write_text(s,encoding="utf-8")
cfg=yaml.safe_load((ROOT/"config/mocap_a.receive_ground.yaml").read_text(encoding="utf-8"))
cfg["output_pose_frame"]="fixed_lab_navigation"
cfg["fixed_reference"]=dict(id="aerolab_world_Y_X_minusZ_v1",
    convention="indoor_navigation_x_reference_y_right_z_down",geographic_north_verified=False,
    magnetic_alignment="disabled",direction_verified_by="operator movement checks 2026-10-21 and 2026-10-23",
    px4_pose_encoding="POSE_FRAME_NED interpreted as fixed indoor navigation, no geographic aiding")
(ROOT/"config/mocap_a.fixed_ground.yaml").write_text(yaml.safe_dump(cfg,sort_keys=False),encoding="utf-8")
print("Fixed laboratory reference implemented with fresh parameter and estimator gates.")
