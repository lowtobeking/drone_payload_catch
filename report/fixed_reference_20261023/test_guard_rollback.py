"""Fault-inject transaction lifecycle; all networking replaced before module import."""
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent
class GuardTests(unittest.TestCase):
    def load_guard(self,root):
        snap=dict(complete=True,version=dict(uid=3544984969635313204),
                  parameters={k:dict(value=v,type=6) for k,v in
                    (("EKF2_EV_CTRL",0),("EKF2_MAG_TYPE",0),("UXRCE_DDS_SYNCT",1))})
        backup=types.ModuleType("backup_parameters_a")
        backup.main=lambda:0
        backup.decode_parameter=lambda v,k,b:struct.unpack("<i",struct.pack("<f",v))[0]
        original=types.ModuleType("receive_parameter_guard_a")
        original.Tap=type("NoNetworkTap",(),{})
        pm=types.ModuleType("pymavlink");pm.mavutil=object()
        spec=importlib.util.spec_from_file_location("isolated_fixed_guard",ROOT/"fixed_guard_a.py")
        mod=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{"backup_parameters_a":backup,"receive_parameter_guard_a":original,"pymavlink":pm}),patch.object(sys,"argv",["fixed_guard",str(root)]),patch("signal.signal"):
            spec.loader.exec_module(mod)
        mod.ROOT=root
        (root/"parameters_backup_passive.json").write_text(json.dumps(snap),encoding="utf-8")
        (root/"configure.request").touch()
        return mod
    def scenario(self,failure):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);mod=self.load_guard(root);events=[];values={"EKF2_EV_CTRL":0,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1}
            class FakeTap:
                boot_ms=20000
                def receive(self): return []
                def ground(self): return True
                def verify_identity(self): events.append("identity")
                def read(self,k): return values[k],6
                def set_checked(self,k,v):
                    events.append((k,v))
                    if failure=="partial_write" and k=="EKF2_MAG_TYPE" and v==5: raise RuntimeError("injected write failure")
                    values[k]=v
                def reboot_fc(self,label):
                    events.append("reboot_"+label)
                    if failure=="restart" and label=="configure": raise RuntimeError("injected missing restart evidence")
                def close(self): events.append("closed")
            mod.Tap=FakeTap
            def proof(tap):
                self.assertEqual(values,{"EKF2_EV_CTRL":9,"EKF2_MAG_TYPE":5,"UXRCE_DDS_SYNCT":1})
                (root/"fixed_parameter_proof.json").write_text("{}",encoding="utf-8")
                (root/"release.request").touch()
            mod.proof=proof
            rc=mod.main()
            result=json.loads((root/"parameter_lease_result.json").read_text(encoding="utf-8"))
            self.assertEqual(values,{"EKF2_EV_CTRL":0,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1})
            self.assertTrue(result["restored"] and result["restore_restart_verified"])
            self.assertFalse((root/"fixed_parameter_proof.json").exists())
            self.assertIn("reboot_restore",events);self.assertEqual(events[-1],"closed")
            self.assertEqual(rc,0 if failure is None else 2)
    def test_success_restore(self): self.scenario(None)
    def test_partial_config_write_still_restores(self): self.scenario("partial_write")
    def test_unverified_restart_still_restores(self): self.scenario("restart")
    def test_allowlist_and_wire_encoding(self):
        with tempfile.TemporaryDirectory() as t:
            mod=self.load_guard(Path(t));tap=mod.Tap.__new__(mod.Tap)
            tap.ground=lambda:True
            writes=[]
            tap.tx=types.SimpleNamespace(param_set_send=lambda *v:writes.append(v))
            tap.receive=lambda:[types.SimpleNamespace(get_type=lambda:"PARAM_VALUE",param_id="EKF2_MAG_TYPE",param_type=6,param_value=struct.unpack("<f",struct.pack("<i",5))[0])]
            tap.read=lambda key:(5,6)
            tap.set_checked("EKF2_MAG_TYPE",5)
            self.assertEqual(struct.unpack("<i",struct.pack("<f",writes[0][3]))[0],5)
            with self.assertRaises(ValueError): tap.set_checked("SYS_AUTOSTART",5)
            with self.assertRaises(ValueError): tap.set_checked("EKF2_MAG_TYPE",1)
            tap.ground=lambda:False
            with self.assertRaises(RuntimeError): tap.set_checked("EKF2_MAG_TYPE",5)
            self.assertEqual(len(writes),1)
if __name__=="__main__": unittest.main()
