"""Explicit vertical-reference contract and fixed shared Z conversion."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from payload_catch.mocap_odometry import load_config,OdometryBuilder,fixed_parameter_issues
class HeightContract(unittest.TestCase):
    def setUp(self):
        self.cfg,self.transform=load_config(ROOT/"config/mocap_a.xyz_ground.yaml")
        self.p=dict(schema=1,jetson_serial="1612425621719",jetson_boot_id="test",
            fc_uid=3544984969635313204,readback_monotonic_us=1000000,fc_restart_verified=True,
            parameters={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":11,"EKF2_HGT_REF":3,"UXRCE_DDS_SYNCT":1})
    def test_xyz_policy_is_explicit_and_xy_rejects_it(self):
        self.assertEqual(fixed_parameter_issues(self.p,1000000,"test",height_fusion=True),[])
        self.assertTrue(fixed_parameter_issues(self.p,1000000,"test"))
    def test_wrong_height_source_or_missing_z_fusion_refused(self):
        for key,val in (("EKF2_HGT_REF",1),("EKF2_EV_CTRL",9),("EKF2_MAG_TYPE",0),("UXRCE_DDS_SYNCT",0)):
            p=copy.deepcopy(self.p);p["parameters"][key]=val
            self.assertTrue(fixed_parameter_issues(p,1000000,"test",height_fusion=True))
        p=copy.deepcopy(self.p);del p["parameters"]["EKF2_HGT_REF"]
        self.assertTrue(fixed_parameter_issues(p,1000000,"test",height_fusion=True))
    def test_height_still_requires_fresh_proof_and_restart(self):
        for key,val in (("readback_monotonic_us",-1000001),("fc_restart_verified",False),("jetson_boot_id","old")):
            p=copy.deepcopy(self.p);p[key]=val
            self.assertTrue(fixed_parameter_issues(p,1000000,"test",height_fusion=True))
    def test_up_is_negative_z_without_changing_shared_origin(self):
        wall=1800000000000000;mono=1000000
        builder=OdometryBuilder(self.cfg,self.transform)
        def msg(z,delta):
            return dict(schema=1,tracker="planeA",sensor=0,frame="world",
                server_stamp_us=wall+43200000000+delta,receive_wall_us=wall+delta,
                receive_monotonic_us=mono+delta,position=[1.,2.,z],quaternion_xyzw=[0.,0.,1.,0.])
        a=builder.accept(msg(.3,0),wall,mono)
        b=builder.accept(msg(.9,30000),wall+30000,mono+30000)
        self.assertAlmostEqual(a["position"][2],-.3)
        self.assertAlmostEqual(b["position"][2]-a["position"][2],-.6)
        self.assertEqual(a["position"][:2],b["position"][:2])
    def test_origin_contract_required(self):
        for change in ({"height_origin":"agl"},{"output_pose_frame":"local_frd"},{"height_fusion":"arbitrary"}):
            cfg=copy.deepcopy(self.cfg);cfg.update(change)
            with tempfile.TemporaryDirectory(dir=ROOT/"report/mocap_z_20261023") as d:
                p=Path(d)/"invalid.yaml";p.write_text(yaml.safe_dump(cfg),encoding="utf-8")
                with self.assertRaises(ValueError):load_config(p)
if __name__=="__main__":unittest.main()
