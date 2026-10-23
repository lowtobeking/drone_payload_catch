"""Exercise real reboot evidence code with 82-second network silence and a young FC."""
import importlib.util
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("guard_tests",ROOT/"test_guard_rollback.py")
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
class RestartEvidence(unittest.TestCase):
    def test_long_network_boot_and_reboot_soon_after_start(self):
        for old_uptime in (490410,85410):
            with tempfile.TemporaryDirectory() as t:
                mod=base.GuardTests().load_guard(Path(t))
                now=[0.]
                class Clock:
                    def monotonic(self):return now[0]
                    def time_ns(self):return int((1000+now[0])*1e9)
                class Fake(mod.Tap):
                    def __init__(self):
                        self.boot_ms=old_uptime;self.boot_mono=0.
                        self.heartbeat_mono=0.;self.land_mono=0.;self.landed=True
                        self.tx=types.SimpleNamespace(command_long_send=lambda *a:None,param_request_read_send=lambda *a:None)
                    def receive(self):
                        now[0]+=.2
                        if now[0]>=82:
                            self.boot_ms=int(now[0]*1000)-1000
                            self.boot_mono=now[0];self.heartbeat_mono=now[0];self.land_mono=now[0]
                        return []
                    def ground(self):return now[0]<.1 or now[0]>=82
                    def close(self):pass
                    def verify_identity(self):pass
                def reopen(tap):
                    tap.tx=types.SimpleNamespace(command_long_send=lambda *a:None,param_request_read_send=lambda *a:None)
                    tap.heartbeat_mono=0.
                with patch.object(mod,"time",Clock()),patch.object(mod.OriginalTap,"__init__",reopen):
                    tap=Fake();tap.reboot_fc("fake")
                self.assertTrue((Path(t)/"fake_reboot_verified.json").exists())
if __name__=="__main__":unittest.main()
