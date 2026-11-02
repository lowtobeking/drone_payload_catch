"""Deterministic proof renewal race test using the actual fixed_issues method."""
import ast
import json
from pathlib import Path
import types
import unittest

PROJECT=Path(__file__).resolve().parents[2]
ROOT=PROJECT/"report/mocap_right_translation_20261026"
def load_function(path,name,globals_dict):
    tree=ast.parse(path.read_text(encoding="utf-8"))
    node=next(node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name==name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(path),"exec"),globals_dict)
    return globals_dict[name]
policy_check=load_function(PROJECT/"payload_catch/mocap_odometry.py","fixed_parameter_issues",{})
state={"now_us":1000}
new_method=load_function(PROJECT/"payload_catch/mocap_odometry_node.py","fixed_issues",{"json":json,"time":types.SimpleNamespace(monotonic_ns=lambda:state["now_us"]*1000),"fixed_parameter_issues":policy_check})
old_method=load_function(ROOT/"mocap_odometry_node.before_proof_clock_fix.py","fixed_issues",{"json":json,"fixed_parameter_issues":policy_check})
class RenewalPath:
    def __init__(self,stamp,complete_at):
        self.stamp=stamp
        self.complete_at=complete_at
    def read_text(self,encoding):
        assert encoding=="utf-8"
        state["now_us"]=self.complete_at
        return json.dumps({"schema":1,"jetson_serial":"1612425621719","jetson_boot_id":"test-boot","fc_uid":3544984969635313204,"readback_monotonic_us":self.stamp,"parameters":{"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":11,"EKF2_HGT_REF":3,"UXRCE_DDS_SYNCT":1},"fc_restart_verified":True})
def instance(stamp=1050,complete_at=1100):
    return types.SimpleNamespace(fixed_mode=True,fixed_fault="",fixed_ready=True,fixed_proof_path=RenewalPath(stamp,complete_at),
        fixed_boot="test-boot",cfg={"height_fusion":"external_vision_reference"},
        fixed_status=(types.SimpleNamespace(cs_mag_hdg=False,cs_mag_3d=False,cs_gnss_pos=False,cs_gnss_vel=False,cs_gnss_yaw=False),1000))
class ProofRace(unittest.TestCase):
    def test_old_reader_misclassifies_concurrent_renewal(self):
        self.assertIn("fixed reference parameter readback stale/future",old_method(instance(),1000))
    def test_new_reader_accepts_renewal_completed_before_validation(self):
        self.assertEqual(new_method(instance(),1000),[])
    def test_new_reader_still_rejects_real_future(self):
        self.assertIn("fixed reference parameter readback stale/future",new_method(instance(stamp=1200),1000))
    def test_new_reader_still_rejects_stale(self):
        self.assertIn("fixed reference parameter readback stale/future",new_method(instance(stamp=1000,complete_at=2001001),1000))
    def test_wrong_boot_remains_rejected(self):
        obj=instance()
        obj.fixed_boot="wrong-boot"
        self.assertIn("fixed reference parameter proof identity mismatch",new_method(obj,1000))
    def test_magnetic_conflict_remains_rejected(self):
        obj=instance()
        obj.fixed_status[0].cs_mag_hdg=True
        self.assertIn("fixed indoor reference conflicts with magnetic/geographic aiding",new_method(obj,1000))
    def test_latched_failure_remains_latched(self):
        obj=instance()
        obj.fixed_fault="existing fault"
        self.assertEqual(new_method(obj,1000),["existing fault"])
if __name__=="__main__":
    unittest.main()
