"""Fixed frame semantics and exact-device fresh parameter proof contracts."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from payload_catch.mocap_odometry import load_config,OdometryBuilder,fixed_parameter_issues
class FixedReferenceTests(unittest.TestCase):
    def setUp(self):
        self.cfg,self.transform=load_config(ROOT/"config/mocap_a.fixed_ground.yaml")
        self.wall,self.mono=1_800_000_000_000_000,1000000
        self.policy=dict(schema=1,jetson_serial="1612425621719",jetson_boot_id="test-boot",
            fc_uid=3544984969635313204,readback_monotonic_us=self.mono,
            parameters={"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1},fc_restart_verified=True)
    def test_fixed_axes_position_orientation_and_unknown_velocity(self):
        b=OdometryBuilder(self.cfg,self.transform)
        out=b.accept(dict(schema=1,tracker="planeA",sensor=0,frame="world",
            server_stamp_us=self.wall+43_200_000_000,receive_wall_us=self.wall,
            receive_monotonic_us=self.mono,position=[1.,2.,3.],quaternion_xyzw=[0.,0.,1.,0.]),self.wall,self.mono)
        self.assertEqual(out["position"],[2.,1.,-3.])
        self.assertEqual(out["pose_frame"],1)
        self.assertAlmostEqual(abs(out["q"][0]),1.)
        self.assertFalse(b.sample_time_verified)
    def test_fixed_policy_fresh_and_same_identity(self):
        self.assertEqual(fixed_parameter_issues(self.policy,self.mono,"test-boot"),[])
        self.assertTrue(fixed_parameter_issues(None,self.mono,"test-boot"))
        for key,value in (("jetson_boot_id","old"),("fc_uid",1),("readback_monotonic_us",self.mono-2000001),
                          ("readback_monotonic_us",self.mono+1),("fc_restart_verified",False)):
            p=copy.deepcopy(self.policy); p[key]=value
            self.assertTrue(fixed_parameter_issues(p,self.mono,"test-boot"))
    def test_magnetic_or_sync_parameter_mismatch_refused(self):
        for key,val in (("EKF2_MAG_TYPE",0),("EKF2_EV_CTRL",1),("UXRCE_DDS_SYNCT",0)):
            p=copy.deepcopy(self.policy); p["parameters"][key]=val
            self.assertTrue(fixed_parameter_issues(p,self.mono,"test-boot"))
    def test_fixed_world_must_not_follow_current_yaw_or_reset_origin(self):
        for key,value in (("world_to_local",[[1,0,0],[0,1,0],[0,0,1]]),
                          ("shared_origin_world_m",[1,2,3])):
            cfg=copy.deepcopy(self.cfg); cfg["transform"][key]=value
            with tempfile.TemporaryDirectory(dir=ROOT/"report/fixed_reference_20261023") as d:
                p=Path(d)/"bad.yaml"; p.write_text(yaml.safe_dump(cfg),encoding="utf-8")
                with self.assertRaises(ValueError): load_config(p)
    def test_unconfirmed_calibrations_remain_false(self):
        for key in ("mounting_confirmed","lever_arm_measured","server_timing_verified","tracking_loss_verified"):
            self.assertIs(self.cfg[key],False)
        self.assertFalse(self.cfg["fixed_reference"]["geographic_north_verified"])
if __name__=="__main__": unittest.main()
