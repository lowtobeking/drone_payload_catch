"""Validate actual shared position/attitude rotation and reject relabel-only NED."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from payload_catch.mocap_odometry import load_config,OdometryBuilder

class HeadingTests(unittest.TestCase):
    def setUp(self):
        self.cfg,self.base=load_config(ROOT/"config/mocap_a.receive_ground.yaml")
        a=-math.pi/4
        self.rz=Rotation.from_euler("z",a).as_matrix()
        base=copy.deepcopy(self.cfg["transform"]["world_to_local"])
        self.cfg["transform"]["world_to_local"]=(self.rz@np.array(base)).tolist()
        self.cfg["output_pose_frame"]="px4_heading_aligned_ned"
        self.cfg["heading_reference"]=dict(method="px4_magnetic_snapshot",north_verified=False,
            yaw_offset_rad=a,std_deg=.1,span_deg=.3,collected_wall_us=time.time_ns()//1000,
            boot_id="a006295d-9ab4-41c6-a277-f0a2455a86a8",base_world_to_local=base)
    def load(self):
        with tempfile.TemporaryDirectory(dir=ROOT/"report/mocap_fusion_20261023") as d:
            p=Path(d)/"candidate.yaml"
            p.write_text(yaml.safe_dump(self.cfg),encoding="utf-8")
            return load_config(p)
    def test_position_and_full_orientation_rotate_together(self):
        cfg,t=self.load()
        pos=[1.,2.,3.]
        q=Rotation.from_euler("xyz",[.2,-.1,.7]).as_quat().tolist()
        p0,q0=self.base.convert(pos,q)
        p1,q1=t.convert(pos,q)
        np.testing.assert_allclose(p1,self.rz@p0,atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(q1).as_matrix(),
            self.rz@Rotation.from_quat(q0).as_matrix(),atol=1e-12)
    def test_relabel_without_rotation_refused(self):
        self.cfg["transform"]["world_to_local"]=self.cfg["heading_reference"]["base_world_to_local"]
        with self.assertRaisesRegex(ValueError,"same measured"):
            self.load()
    def test_missing_reference_refused(self):
        del self.cfg["heading_reference"]
        with self.assertRaises(ValueError): self.load()
    def test_cannot_claim_north_verified(self):
        self.cfg["heading_reference"]["north_verified"]=True
        with self.assertRaises(ValueError): self.load()
    def test_default_frd_unchanged(self):
        cfg,t=load_config(ROOT/"config/mocap_a.receive_ground.yaml")
        b=OdometryBuilder(cfg,t)
        wall,mono=1_800_000_000_000_000,1000000
        out=b.accept(dict(schema=1,tracker="planeA",sensor=0,frame="world",
            server_stamp_us=wall+43_200_000_000,receive_wall_us=wall,
            receive_monotonic_us=mono,position=[1.,2.,3.],quaternion_xyzw=[0.,0.,1.,0.]),wall,mono)
        self.assertEqual(out["pose_frame"],2)
        self.assertEqual(out["position"],[2.,1.,-3.])
    def test_candidate_frame_and_no_fake_calibrations(self):
        cfg,t=self.load()
        b=OdometryBuilder(cfg,t)
        wall,mono=1_800_000_000_000_000,1000000
        out=b.accept(dict(schema=1,tracker="planeA",sensor=0,frame="world",
            server_stamp_us=wall+43_200_000_000,receive_wall_us=wall,
            receive_monotonic_us=mono,position=[1.,2.,3.],quaternion_xyzw=[0.,0.,1.,0.]),wall,mono)
        self.assertEqual(out["pose_frame"],1)
        self.assertFalse(b.sample_time_verified)
        self.assertFalse(cfg["lever_arm_measured"])
        self.assertFalse(cfg["mounting_confirmed"])
if __name__=="__main__": unittest.main()
