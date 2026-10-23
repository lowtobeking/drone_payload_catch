"""Exercise actual batch proof parsing and rejection with mocked MAVLink packets."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("base_guard_tests",ROOT/"test_guard_rollback.py")
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
class BatchProof(unittest.TestCase):
    def run_case(self,wrong=False):
        with tempfile.TemporaryDirectory() as t:
            m=b.GuardTests().load_guard(Path(t));sent=[]
            class Tap:
                tx=types.SimpleNamespace(param_request_read_send=lambda *args:sent.append(args))
                def receive(self):
                    return [types.SimpleNamespace(get_type=lambda:"PARAM_VALUE",param_id=k,param_type=6,
                        param_value=struct.unpack("<f",struct.pack("<i",v))[0])
                        for k,v in (("EKF2_MAG_TYPE",5),("EKF2_EV_CTRL",0 if wrong else 9),("UXRCE_DDS_SYNCT",1))]
                def ground(self):return True
            with patch.object(m.Path,"read_text",return_value="isolated-boot"):
                if wrong:
                    with self.assertRaises(RuntimeError):m.proof(Tap())
                else:m.proof(Tap())
            if not wrong:
                data=json.loads((Path(t)/"fixed_parameter_proof.json").read_text())
                self.assertEqual(data["parameters"],{"EKF2_MAG_TYPE":5,"EKF2_EV_CTRL":9,"UXRCE_DDS_SYNCT":1})
                self.assertEqual(len(sent),3)
    def test_three_readbacks(self):self.run_case()
    def test_unexpected_parameter_refused(self):self.run_case(True)
if __name__=="__main__":unittest.main()
