#!/usr/bin/env python3
"""Fault injection for latch interlock and independent PWM worker; no hardware I/O."""
import json
import multiprocessing as mp
from pathlib import Path
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from unittest.mock import Mock, call

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_catch.payload_servo import ServoConfig, ServoInterlock, AttemptRecord
from payload_catch.servo_pwm import MockPWM, JetsonPWM, _worker


def supervisor(wall=100, session='flight', **changes):
    return dict(stamp=wall, session=session, permits={'0': True, '1': True},
                actions={'0': 'run', '1': 'run'}, release_permitted=True, **changes)


def feedback(wall=100, **changes):
    data = dict(stamp=wall, session='flight', locked=True, present=True, detached=False, healthy=True)
    data.update(changes)
    return data


def authorized(wall=100, **changes):
    data = dict(stamp=wall, session='flight', owner=0, nonce='unique', command_at=wall, planned_at=wall + .08)
    data.update(changes)
    return data


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'attempt.json'
        self.backend = MockPWM()
        self.cfg = ServoConfig(enabled=True, calibrated=True, release_delay_s=.08)
        self.core = ServoInterlock(self.cfg, self.backend, AttemptRecord(self.path), mock=True)
        self.refresh()

    def refresh(self, wall=100, mono=10):
        self.core.on_supervisor(supervisor(wall), wall, mono)
        self.core.on_feedback(feedback(wall), wall, mono)
        self.core.tick(wall, mono)

    def test_candidate_configuration_does_not_arm_hardware(self):
        for cfg in (ServoConfig(), replace(self.cfg, calibrated=False)):
            core = ServoInterlock(cfg, MockPWM(), AttemptRecord(self.path))
            core.on_supervisor(supervisor(), 100, 10)
            core.on_feedback(feedback(), 100, 10)
            self.assertFalse(core.ready(100, 10))
            self.assertFalse(core.authorize(authorized(), 100, 10))
            self.assertFalse(self.path.exists())

    def test_one_shot_command_is_not_physical_detachment(self):
        self.assertTrue(self.core.authorize(authorized(), 100, 10))
        self.assertEqual(self.backend.pulses, [1700])
        self.assertEqual(self.core.state, 'COMMAND_SENT')
        self.assertFalse(self.core.physical_released)
        self.assertTrue(self.core.hardware_ready(100.04, 10.04))
        self.assertFalse(self.core.authorize(authorized(100.01), 100.01, 10.01))
        self.assertEqual(len(self.backend.pulses), 1)

    def test_opening_stops_if_either_aircraft_permission_or_action_changes(self):
        for key,value in (('permits',False),('actions','hold')):
            with self.subTest(key=key):
                backend=MockPWM()
                core=ServoInterlock(self.cfg,backend,AttemptRecord(Path(self.tmp.name)/(key+'.json')),mock=True)
                core.on_supervisor(supervisor(),100,10)
                core.on_feedback(feedback(),100,10)
                core.tick(100,10)
                self.assertTrue(core.authorize(authorized(),100,10))
                state=supervisor(100.01)
                state[key]['1']=value
                core.on_supervisor(state,100.01,10.01)
                core.on_feedback(feedback(100.01),100.01,10.01)
                core.tick(100.01,10.01)
                self.assertEqual(core.state,'FAULT')
                self.assertFalse(core.physical_released)

    def test_measured_detachment_is_confirmed_idempotently(self):
        self.core.authorize(authorized(), 100, 10)
        for stamp in (100.08, 100.1, 100.13):
            self.core.on_feedback(feedback(stamp, locked=False, present=False, detached=True,
                                           detached_at=100.08), stamp, stamp - 90)
        self.assertEqual(self.core.state, 'RELEASED')
        self.assertTrue(self.core.physical_released)

    def test_bad_timing_still_reports_actual_detachment(self):
        self.core.authorize(authorized(), 100, 10)
        self.core.on_feedback(feedback(100.04, locked=False, present=False, detached=True,
                                       detached_at=100.04), 100.04, 10.04)
        self.assertTrue(self.core.physical_released)
        self.assertEqual(self.core.state, 'FAULT')

    def test_completed_release_survives_landing_stop_without_retry(self):
        self.assertTrue(self.core.authorize(authorized(),100,10))
        self.core.on_feedback(feedback(100.08,locked=False,present=False,detached=True,
            detached_at=100.08),100.08,10.08)
        self.assertEqual(self.core.state,'RELEASED')
        self.backend.close()
        for index,action in enumerate(('hold','land','kill','manual')):
            wall=100.09+index*.01
            state=supervisor(wall);state['actions']['0']=action
            self.core.on_supervisor(state,wall,wall-90)
            self.core.abort()
            self.assertEqual(self.core.state,'RELEASED')
            self.assertEqual(self.core.reason,'')
            self.assertTrue(self.path.exists())
            self.assertFalse(self.core.authorize(authorized(wall),wall,wall-90))
        self.assertEqual(self.backend.pulses,[1700])

    def test_landing_does_not_clear_timing_fault_after_detachment(self):
        self.assertTrue(self.core.authorize(authorized(),100,10))
        self.core.on_feedback(feedback(100.11,locked=False,present=False,detached=True,
            detached_at=100.11),100.11,10.11)
        reason=self.core.reason
        self.assertTrue(self.core.physical_released)
        self.assertEqual(self.core.state,'FAULT')
        self.core.abort()
        self.assertEqual(self.core.state,'FAULT')
        self.assertEqual(self.core.reason,reason)

    def test_cancel_before_open_blocks_new_authorizations(self):
        self.core.abort()
        self.refresh(100.01, 10.01)
        self.assertFalse(self.core.authorize(authorized(100.01), 100.01, 10.01))
        self.assertFalse(self.backend.pulses)

    def test_abort_after_open_never_relocks_and_still_observes_release(self):
        self.core.authorize(authorized(), 100, 10)
        self.core.abort()
        self.core.on_feedback(feedback(100.08, locked=False, present=False, detached=True,
                                       detached_at=100.08), 100.08, 10.08)
        self.assertEqual(self.backend.pulses, [1700])
        self.assertTrue(self.core.physical_released)
        self.assertEqual(self.core.state, 'FAULT')

    def test_replay_owner_session_and_timing_rejected(self):
        for changes in ({'stamp': 99.8}, {'stamp': 100.1}, {'owner': 1}, {'session': 'old'},
                        {'nonce': ''}, {'planned_at': 100.2}, {'command_at': 100.1},
                        {'planned_at': float('nan')}, {'command_at': 99.9}):
            with self.subTest(changes=changes):
                core = ServoInterlock(self.cfg, MockPWM(), AttemptRecord(self.path), mock=True)
                core.on_supervisor(supervisor(), 100, 10)
                core.on_feedback(feedback(), 100, 10)
                self.assertFalse(core.authorize(authorized(**changes), 100, 10))
                self.assertFalse(self.path.exists())

    def test_duplicate_supervisor_and_feedback_do_not_refresh(self):
        self.core.on_supervisor(supervisor(), 100.19, 10.19)
        self.core.on_feedback(feedback(), 100.19, 10.19)
        self.core.tick(100.21, 10.21)
        self.assertEqual(self.core.state, 'FAULT')
        self.assertFalse(self.core.authorize(authorized(100.21), 100.21, 10.21))

    def test_permission_revoked_without_action_change(self):
        s = supervisor(100.01)
        s['permits']['1'] = False
        self.core.on_supervisor(s, 100.01, 10.01)
        self.assertFalse(self.core.authorize(authorized(100.01), 100.01, 10.01))

    def test_supervisor_restart_latches_stop(self):
        self.core.on_supervisor(supervisor(100.01, session='new'), 100.01, 10.01)
        self.assertEqual(self.core.state, 'FAULT')

    def test_missing_inconsistent_or_unhealthy_feedback(self):
        for changes in ({'locked': False}, {'healthy': False}, {'present': 1},
                        {'session': 'other'}, {'stamp': float('nan')}):
            self.setUp()
            self.core.on_feedback(feedback(100.01, **changes), 100.01, 10.01)
            self.assertFalse(self.core.authorize(authorized(100.01), 100.01, 10.01))

    def test_sensor_edge_required_for_physical_release(self):
        self.core.authorize(authorized(), 100, 10)
        self.core.on_feedback(feedback(100.08, locked=False, present=False, detached=True), 100.08, 10.08)
        self.assertFalse(self.core.physical_released)
        self.core.tick(100.08, 10.08)
        self.assertEqual(self.core.state, 'FAULT')

    def test_write_failure_consumes_attempt_and_restart_blocks(self):
        self.backend.open = Mock(side_effect=OSError('readback error'))
        self.assertFalse(self.core.authorize(authorized(), 100, 10))
        self.assertTrue(self.path.exists())
        restarted = ServoInterlock(self.cfg, MockPWM(), AttemptRecord(self.path), mock=True)
        self.assertEqual(restarted.state, 'FAULT')
        self.assertFalse(restarted.authorize(authorized(100.1), 100.1, 10.1))

    def test_successful_attempt_and_even_partial_record_survive_restart(self):
        self.core.authorize(authorized(), 100, 10)
        self.assertEqual(json.loads(self.path.read_text())['nonce'], 'unique')
        self.path.write_text('{')
        restarted = ServoInterlock(self.cfg, MockPWM(), AttemptRecord(self.path), mock=True)
        self.assertEqual(restarted.state, 'FAULT')

    def test_feedback_timeout_and_stall_during_motion(self):
        self.core.authorize(authorized(), 100, 10)
        self.core.tick(100.16, 10.16)
        self.assertEqual(self.core.state, 'FAULT')
        self.assertFalse(self.core.physical_released)
        self.assertEqual(self.backend.pulses, [1700])

    def test_no_detachment_timeout_with_fresh_heartbeats(self):
        self.core.authorize(authorized(), 100, 10)
        self.core.on_supervisor(supervisor(100.14), 100.14, 10.14)
        self.core.on_feedback(feedback(100.14), 100.14, 10.14)
        self.core.tick(100.14, 10.14)
        self.assertIn('not confirmed', self.core.reason)

    def test_invalid_configs_fail_closed(self):
        for changes in ({'open_us': 2000}, {'neutral_us': 1700}, {'period_ns': 3003003},
                        {'release_delay_s': float('nan')}, {'delay_tolerance_s': .5},
                        {'enabled': 'false'}, {'calibrated': 'true'}, {'owner': False},
                        {'max_output_s': .1},
                        {'delay_reference': 'gui_apply'}, {'delay_reference': 'unmeasured_direct'}, {'delay_reference': 'operator_direct'},
                        {'delay_reference': 'local_command'}, {'release_delay_s': True}):
            with self.assertRaises(ValueError):
                replace(self.cfg, **changes).validate()


    def test_one_point_five_second_opening_waits_for_measured_detachment(self):
        self.cfg = replace(self.cfg, release_delay_s=1.5)
        self.core = ServoInterlock(self.cfg, self.backend, AttemptRecord(self.path), mock=True)
        self.refresh()
        self.assertTrue(self.core.authorize(authorized(planned_at=101.5), 100, 10))
        for step in range(1,31):
            wall, mono = 100+step*.05, 10+step*.05
            self.refresh(wall, mono)
            self.assertEqual(self.core.state, 'COMMAND_SENT')
            self.assertTrue(self.core.hardware_ready(wall,mono))
            self.assertFalse(self.core.physical_released)  # A timer is never a sensor.
        self.core.on_feedback(feedback(101.501, locked=False, present=False, detached=True,
                                      detached_at=101.5), 101.501, 11.501)
        self.assertEqual(self.core.state, 'RELEASED')
        self.assertTrue(self.core.physical_released)
        self.assertEqual(self.backend.pulses, [1700])

    def test_long_opening_times_out_without_detachment_despite_fresh_sensors(self):
        self.cfg = replace(self.cfg, release_delay_s=1.5)
        self.core = ServoInterlock(self.cfg, self.backend, AttemptRecord(self.path), mock=True)
        self.refresh()
        self.assertTrue(self.core.authorize(authorized(planned_at=101.5), 100, 10))
        for step in range(1,32):
            self.refresh(100+step*.05,10+step*.05)
        self.refresh(101.56,11.56)
        self.assertEqual(self.core.state, 'FAULT')
        self.assertIn('not confirmed',self.core.reason)
        self.assertFalse(self.core.physical_released)
        self.assertTrue(self.path.exists())

    def test_operator_abort_partway_through_long_opening_never_retries(self):
        self.cfg = replace(self.cfg, release_delay_s=1.5)
        self.core = ServoInterlock(self.cfg, self.backend, AttemptRecord(self.path), mock=True)
        self.refresh()
        self.core.authorize(authorized(planned_at=101.5),100,10)
        for step in range(1,21):self.refresh(100+step*.05,10+step*.05)
        self.core.abort()
        self.refresh(101.01,11.01)
        self.assertFalse(self.core.authorize(authorized(101.01,planned_at=102.51),101.01,11.01))
        self.assertEqual(self.backend.pulses,[1700])
        self.assertEqual(self.core.state,'FAULT')

    def test_old_ramp_measurement_cannot_be_promoted_to_direct_flight_calibration(self):
        import yaml
        path=Path(__file__).resolve().parents[1]/'config/payload_servo.yaml'
        raw=yaml.safe_load(path.read_text())['payload_servo']['ros__parameters']
        cfg=ServoConfig(**{k:v for k,v in raw.items() if k in ServoConfig.__dataclass_fields__})
        cfg.validate()
        self.assertEqual((cfg.neutral_us,cfg.open_us,cfg.release_delay_s),(1500,1700,1.5))
        self.assertFalse(cfg.enabled or cfg.calibrated)
        self.assertEqual(cfg.delay_reference,'operator_direct')
        self.assertEqual(cfg.confirmation_mode,'timed')
        replace(cfg,enabled=True,calibrated=True).validate()  # Explicit operator-direct timed profile.
        with self.assertRaisesRegex(ValueError,'local-command'):
            replace(cfg,confirmation_mode='sensor',enabled=True,calibrated=True).validate()

    def test_write_receipt_and_sensor_edge_keep_separate_clocks(self):
        receipt=dict(write_started_at=100.005,write_completed_at=100.009)
        self.backend.open=Mock(return_value=receipt)
        self.assertTrue(self.core.authorize(authorized(),100,10))
        self.assertEqual(self.core.scheduled_command_at,100)
        self.assertEqual(self.core.output_written_at,100.009)
        self.backend.open.assert_called_once_with(1700,latest_start_at=100.02)
        self.core.on_feedback(feedback(100.10,locked=False,present=False,detached=True,
                                      detached_at=100.08),100.10,10.1)
        self.assertEqual(self.core.detached_at,100.08)
        self.assertTrue(self.core.physical_released)

    def test_late_or_missing_real_write_receipt_faults_and_consumes_attempt(self):
        for receipt in (None,dict(write_started_at=100.01,write_completed_at=100.04)):
            with self.subTest(receipt=receipt):
                path=Path(self.tmp.name)/('receipt_'+str(receipt is None))
                backend=MockPWM();backend.open=Mock(return_value=receipt)
                core=ServoInterlock(self.cfg,backend,AttemptRecord(path),mock=False)
                core.on_supervisor(supervisor(),100,10)
                core.on_feedback(feedback(),100,10)
                self.assertFalse(core.authorize(authorized(),100,10))
                self.assertEqual(core.state,'FAULT');self.assertTrue(path.exists())


class DirectPulseTests(unittest.TestCase):
    def test_first_hardware_write_is_1700_without_neutral_or_ramp(self):
        backend=JetsonPWM();backend.owned=True;backend.write=Mock()
        backend.open(1700)
        self.assertEqual(backend.write.call_args_list,
                         [call("duty_cycle",1700000),call("enable",1)])

    def test_worker_and_hardware_write_limits_reject_outside_1200_1800(self):
        backend=JetsonPWM();backend.owned=True;backend.write=Mock()
        for pulse in (1199,1801,True,1700.5):
            with self.assertRaises(RuntimeError):backend.open(pulse)
        backend.write.assert_not_called()
        for pulse in (1200,1800):backend.open(pulse)


class OwnedPath:
    def __init__(self, owner, name='chip'):
        self.owner, self.name = owner, name
    def __truediv__(self, name):
        return OwnedPath(self.owner, self.name + '/' + name)
    def exists(self):
        return self.owner.exported
    def write_text(self, value):
        self.owner.events.append(('unexport', value))
        self.owner.exported = False


class RollbackTests(unittest.TestCase):
    def test_cleanup_attempts_register_restore_after_disable_failure(self):
        backend = JetsonPWM()
        backend.owned = True
        backend.original = {'value': 0x54}
        backend.exported = True
        backend.events = []
        backend.chip = OwnedPath(backend)
        backend.lock = Mock()
        backend.journal = Mock()
        def write(name, value):
            backend.events.append((name, value))
            if name == 'enable':
                raise OSError('disable readback failed')
        backend.write = write
        backend.register = lambda value=None: backend.events.append(('register', value)) or 0x54
        with self.assertRaises(RuntimeError):
            backend.close()
        self.assertIn(('register', 0x54), backend.events)
        self.assertIn(('unexport', '0'), backend.events)
        backend.journal.unlink.assert_not_called()


class FakeWorkerBackend:
    events = None
    def prepare(self, period):
        self.events.put('prepared')
    def open(self, pulse):
        self.events.put(('pulse', pulse))
    def close(self):
        self.events.put('restored')


def fake_worker(child, parent, events):
    import payload_catch.servo_pwm as module
    parent.close()
    FakeWorkerBackend.events = events
    module.JetsonPWM = FakeWorkerBackend
    module._worker(child, 20000000)


@unittest.skipUnless('fork' in mp.get_all_start_methods(), 'worker process injection uses Linux fork')
class WatchdogTests(unittest.TestCase):
    def start(self):
        ctx = mp.get_context('fork')
        parent, child = ctx.Pipe()
        events = ctx.Queue()
        proc = ctx.Process(target=fake_worker, args=(child, parent, events))
        proc.start()
        child.close()
        self.addCleanup(parent.close)
        self.addCleanup(lambda: proc.terminate() if proc.is_alive() else None)
        self.assertTrue(parent.poll(2))
        self.assertEqual(parent.recv()[0], 'ready')
        self.assertEqual(events.get(timeout=1), 'prepared')
        return parent, proc, events

    def test_parent_hang_rolls_back_within_watchdog_bound(self):
        parent, proc, events = self.start()
        parent.send(('open', 1600, time.monotonic()))
        self.assertTrue(parent.poll(1))
        self.assertEqual(parent.recv()[0], 'open')
        self.assertEqual(events.get(timeout=1), ('pulse', 1600))
        proc.join(1)
        self.assertFalse(proc.is_alive())
        while True:
            event = events.get(timeout=1)
            if event == 'restored':break

    def test_parent_disconnect_rolls_back(self):
        parent, proc, events = self.start()
        parent.close()
        proc.join(1)
        self.assertFalse(proc.is_alive())
        while True:
            event = events.get(timeout=1)
            if event == 'restored':break


    def test_fresh_heartbeats_allow_long_opening_but_cannot_bypass_output_limit(self):
        parent,proc,events=self.start()
        parent.send(('open',1700,time.monotonic()))
        self.assertTrue(parent.poll(1));self.assertEqual(parent.recv()[0],'open')
        started=time.monotonic();at_one_point_five=False;errors=[]
        while proc.is_alive() and time.monotonic()-started < 3:
            try:
                parent.send(('ping',None,time.monotonic()))
            except (BrokenPipeError, EOFError, ConnectionResetError):
                # Worker may enforce its deadline between is_alive and send.
                # Verify its explicit timeout event and cleanup below.
                break
            if parent.poll(.1):
                try:
                    event,reason=parent.recv()
                except (EOFError,ConnectionResetError):break
                if event=='error':
                    errors.append(reason)
                    break
            if time.monotonic()-started >= 1.5:at_one_point_five=True
            time.sleep(.02)
        proc.join(1)
        self.assertTrue(at_one_point_five)
        self.assertFalse(proc.is_alive())
        self.assertGreaterEqual(time.monotonic()-started,1.9)
        self.assertLess(time.monotonic()-started,2.8)
        while parent.poll():
            try:
                event,reason=parent.recv()
            except (EOFError,ConnectionResetError):break
            if event=='error':errors.append(reason)
        self.assertTrue(any('maximum PWM' in reason for reason in errors),errors)
        pulses=[]
        while True:
            event=events.get(timeout=1)
            if event=='restored':break
            if isinstance(event,tuple):pulses.append(event[1])
        self.assertEqual(pulses,[1700])  # First write only; never 1500 or intermediate values.


    def test_expired_write_deadline_never_outputs(self):
        parent,proc,events=self.start()
        parent.send(('open',dict(pulse_us=1700,latest_start_at=time.time()-.1),time.monotonic()))
        self.assertTrue(parent.poll(1));event,reason=parent.recv()
        self.assertEqual(event,'error');self.assertIn('deadline expired',reason)
        proc.join(1);self.assertFalse(proc.is_alive())
        self.assertEqual(events.get(timeout=1),'restored')

    def test_successful_open_ack_brackets_write_time(self):
        parent,proc,events=self.start()
        start=time.time()
        parent.send(('open',dict(pulse_us=1700,latest_start_at=start+.2),time.monotonic()))
        self.assertTrue(parent.poll(1));event,receipt=parent.recv()
        self.assertEqual(event,'open')
        self.assertLessEqual(start,receipt['write_started_at'])
        self.assertLessEqual(receipt['write_started_at'],receipt['write_completed_at'])
        self.assertLessEqual(receipt['write_completed_at'],time.time())
        self.assertEqual(events.get(timeout=1),('pulse',1700))
        parent.send(('close',None,time.monotonic()));proc.join(1)

    def test_stale_command_cannot_move(self):
        parent, proc, events = self.start()
        parent.send(('open', 1600, time.monotonic() - 1))
        proc.join(1)
        self.assertFalse(proc.is_alive())
        while True:
            event = events.get(timeout=1)
            if event == 'restored':break


if __name__ == '__main__':
    unittest.main()
