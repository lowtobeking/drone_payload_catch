#!/usr/bin/env python3
"""PX4 parameter snapshot protocol checks; no network or FC required."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import patch
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.a_solo_preflight import live_parameters

def frame(kind, **fields):
    return NS(kind=kind, get_srcSystem=lambda:1, get_srcComponent=lambda:1, **fields)

def param(key,index,value=1.,count=2,kind=9):
    return frame("PARAM_VALUE",param_id=key,param_index=index,
                 param_count=count,param_type=kind,param_value=value)

class FakeLink:
    def __init__(self,messages):
        self.messages=list(messages)
        self.mav=NS(heartbeat_send=lambda *a:None,command_long_send=lambda *a:None,
                    param_request_list_send=lambda *a:None)
        self.closed=False
    def recv_match(self, **kw):
        if self.messages:
            msg=self.messages.pop(0)
            if msg.kind != kw["type"]:
                raise AssertionError("test stream order")
            return msg
        return None
    def close(self):self.closed=True

class ProtocolTests(unittest.TestCase):
    def read(self,params,heartbeats=None):
        heartbeats=heartbeats or [frame("HEARTBEAT",type=2,autopilot=12)]
        link=FakeLink(heartbeats+[frame("AUTOPILOT_VERSION",capabilities=1)]+params)
        module=ModuleType("pymavlink")
        module.mavutil=NS(mavlink_connection=lambda *a,**kw:link,
            mavlink=NS(MAV_TYPE_GCS=6,MAV_AUTOPILOT_INVALID=8,
                       MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE=1,
                       MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST=2))
        tick=iter(i*.1 for i in range(2000))
        try:
            with patch.dict(sys.modules,{"pymavlink":module}),patch(
                    "tools.a_solo_preflight.time.monotonic",side_effect=lambda:next(tick)):
                return live_parameters("test:no_network")
        finally:self.assertTrue(link.closed)
    def test_px4_hash_metadata(self):
        self.assertEqual(self.read([param("_HASH_CHECK",65535,float("nan"),kind=5),
            param("ONE",0),param("TWO",1,2.)]),{"ONE":1.,"TWO":2.})
    def test_forwarded_gcs_heartbeat(self):
        self.assertEqual(len(self.read([param("ONE",0),param("TWO",1)],
            [frame("HEARTBEAT",type=6,autopilot=8),
             frame("HEARTBEAT",type=2,autopilot=12)])),2)
    def test_foreign_fc_rejected(self):
        foreign=frame("HEARTBEAT",type=2,autopilot=12)
        foreign.get_srcSystem=lambda:2
        with self.assertRaisesRegex(RuntimeError,"not PX4"):
            self.read([], [foreign])
    def test_malformed_hash_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"hash metadata"):
            self.read([param("_HASH_CHECK",0,kind=5)])
    def test_other_invalid_index_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"index invalid"):
            self.read([param("UNKNOWN",65535)])
    def test_changed_count_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"count changed"):
            self.read([param("_HASH_CHECK",65535,kind=5),param("ONE",0,count=3)])
    def test_missing_parameter_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"incomplete"):
            self.read([param("_HASH_CHECK",65535,kind=5),param("ONE",0)])

if __name__=="__main__":unittest.main(verbosity=2)
