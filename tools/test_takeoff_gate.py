#!/usr/bin/env python3
"""Runtime takeoff gate integration tests on actual methods; never initializes ROS."""
import ast
import json
import sys
import types
import unittest
import __future__
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.preflight_health import (TakeoffGate, HealthAccumulator, evaluate,
    approval_reason, read_approval, EST_BAD, FAILSAFE_FATAL)
from tools.test_live_probe import healthy
from tools.preflight_check import main as preflight_main
from payload_catch.fcu_params import check_params

N = types.SimpleNamespace


def packet(key, timestamp):
    data = dict(timestamp=timestamp)
    if key == 'lpos':
        data.update(x=0., y=0., z=0., vx=0., vy=0., vz=0., xy_valid=True,
                    z_valid=True, v_xy_valid=True, v_z_valid=True,
                    heading_good_for_control=True, dead_reckoning=False, eph=.15, epv=.20)
    elif key == 'att': data.update(q=[1.,0.,0.,0.])
    elif key == 'status': data.update(pre_flight_checks_pass=True)
    elif key == 'failsafe':
        data.update({k: False for k, _ in FAILSAFE_FATAL})
        data['offboard_control_signal_lost'] = True
    elif key == 'est': data.update({**{k: False for k in EST_BAD},
                                  'cs_tilt_align': True, 'cs_yaw_align': True})
    elif key == 'batt': data.update(connected=True, warning=0, remaining=.8)
    elif key == 'gps': data.update(eph=.9, epv=1.2, satellites_used=25)
    elif key == 'rc': data.update(valid=True,data_source=1)
    return N(**data)


def feed(gate, now, bad_peer=None):
    for i, acc in gate.health.items():
        for key in acc.samples:
            m = packet(key, int(now * 1e6) + 1)
            if key == 'batt' and i == bad_peer: m.remaining = .39
            acc.record(key, m, now)


def receipt(now=100):
    return dict(schema=1,passed=True,checked_at=now,drone_ids=[0,1],
                battery_min=.4,gps=True,params_checked=[0,1])


def actual_method(name):
    file = ROOT / 'payload_catch/px4_iface.py'
    cls = next(n for n in ast.parse(file.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
    node = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name == name)
    ns = {'time': N(monotonic=lambda:100), 'read_approval': lambda *a,**kw:'',
          'VehicleCommand': N(VEHICLE_CMD_DO_SET_MODE=176,VEHICLE_CMD_COMPONENT_ARM_DISARM=400)}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec',
                 flags=__future__.annotations.compiler_flag), ns)
    return ns[name]


class GateTests(unittest.TestCase):
    def test_both_aircraft_and_continuous_good_window(self):
        gate = TakeoffGate()
        for now in range(1, 6):
            feed(gate, now)
            ready, _ = gate.check(now)
            self.assertEqual(ready, now >= 5)
        feed(gate, 6, bad_peer=1)
        self.assertFalse(gate.check(6)[0])
        self.assertIsNone(gate.good_since)
        self.assertFalse(gate.check(100)[0])

    def test_duplicates_out_of_order_and_frozen_source_age(self):
        acc = HealthAccumulator()
        m = packet('batt', 100)
        acc.record('batt',m,0)
        acc.record('batt',m,1)
        acc.record('batt',packet('batt',99),2)
        self.assertEqual(acc.snapshot(2).n_batt,1)
        self.assertEqual(acc.snapshot(2).ages['batt'],2)
        self.assertEqual(acc.snapshot(4).n_batt,0)

    def test_invalid_battery_and_lost_validity_not_hidden(self):
        gate = TakeoffGate(require_gps=False)
        feed(gate,1);feed(gate,2)
        m = packet('batt',3000001);m.remaining=float('nan')
        gate.health[0].record('batt',m,3)
        self.assertFalse(gate.check(3)[0])
        m = packet('lpos',3000002);m.vx=float('nan')
        gate.health[1].record('lpos',m,3)
        self.assertIn('lpos.vx',gate.health[1].snapshot(3).invalid_data)

    def test_receipt_freshness_completeness_and_no_silent_skip(self):
        self.assertEqual(approval_reason(receipt(),now=100),'')
        for changes in ({'checked_at':39}, {'checked_at':101}, {'params_checked':[0]},
                        {'battery_min':.15}, {'passed':False}, {'gps':False}):
            self.assertTrue(approval_reason({**receipt(),**changes},now=100))
        self.assertTrue(read_approval('/definitely/missing/preflight.json'))
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/'approval.json';path.write_text('not JSON')
            self.assertTrue(read_approval(path,now=100))

    def test_no_control_commands_or_heartbeat_when_blocked(self):
        for name in ('_tick','arm_and_offboard'):
            events=[]
            obj=N(_killed=False,_landing=False,_offboard_confirmed=False,
                  _task_report=lambda:None,_ever_offboard=False,_operator_paused=False,
                  _takeoff_ready=lambda:False,auto_arm=True,_armed=False,_nav_state=0,
                  send_command=lambda *a:events.append('command'),
                  broadcast_offboard_mode=lambda:events.append('heartbeat'),
                  control=lambda:events.append('task'))
            actual_method(name)(obj)
            self.assertEqual(events,[])

    def test_authorized_but_not_confirmed_never_starts_task(self):
        events=[]
        obj=N(_killed=False,_landing=False,_offboard_confirmed=False,_tick_count=1,hz=50,
              _task_report=lambda:None,_ever_offboard=False,_operator_paused=False,
              _takeoff_ready=lambda:True,broadcast_offboard_mode=lambda:events.append('heartbeat'),
              publish_velocity=lambda *a,**kw:events.append('zero_setpoint'),yaw=0,
              control=lambda:events.append('task'))
        actual_method('_tick')(obj)
        self.assertEqual(events,['heartbeat','zero_setpoint'])

    def test_40_percent_not_used_as_inflight_landing_threshold(self):
        events=[]
        obj=N(_killed=False,_landing=False,_offboard_confirmed=True,_tick_count=1,hz=50,
              _task_report=lambda:None,_ever_offboard=False,_operator_paused=False,_safety_state='OK',
              _takeoff_ready=lambda:False,broadcast_offboard_mode=lambda:None,
              get_clock=lambda:N(now=lambda:N(nanoseconds=100000000000)),
              _safety_update=lambda now:None,control=lambda:events.append('task'))
        actual_method('_tick')(obj)
        self.assertEqual(events,['task'])

    def test_approval_not_written_if_either_live_or_param_checks_fail(self):
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/'approval.json'
            args=['--live','--param-mavlink-a','a','--param-mavlink-b','b',
                  '--write-approval',str(path)]
            from tools.preflight_check import Res
            for failing in ('live','params'):
                path.write_text(json.dumps(receipt()))
                live=[Res('d0 battery','fail' if failing=='live' else 'ok')]
                params=[Res('params','fail' if failing=='params' else 'ok')]
                with patch('tools.preflight_check.check_environment',return_value=[]), \
                     patch('tools.preflight_check.probe_live',return_value=live), \
                     patch('tools.preflight_check.probe_params',return_value=params):
                    self.assertEqual(preflight_main(args),1)
                self.assertFalse(path.exists())

    def test_strict_params_reject_missing_and_nan(self):
        self.assertTrue(any(i.level=='fail' for i in check_params({},require_complete=True)))
        self.assertTrue(any(i.level=='fail' for i in check_params(
            {'MPC_THR_MIN':float('nan')},require_complete=True)))

    def test_receipt_written_only_after_both_parameter_checks(self):
        from tools.preflight_check import Res
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/'approval.json'
            args=['--live','--gps','--param-mavlink-a','a','--param-mavlink-b','b',
                  '--write-approval',str(path)]
            with patch('tools.preflight_check.check_environment',return_value=[]), \
                 patch('tools.preflight_check.probe_live',return_value=[Res('live','ok')]), \
                 patch('tools.preflight_check.probe_params',return_value=[Res('params','ok')]) as probe:
                self.assertEqual(preflight_main(args),0)
            result=json.loads(path.read_text())
            self.assertEqual(result['params_checked'],[0,1])
            self.assertEqual([c.kwargs['system_id'] for c in probe.call_args_list],[1,2])
            self.assertEqual(approval_reason(result),'')


if __name__ == '__main__':
    unittest.main()
