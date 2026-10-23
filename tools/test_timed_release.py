from pathlib import Path
import json
import sys
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from payload_catch.payload_servo import ServoConfig,ServoInterlock,AttemptRecord,servo_signature
from payload_catch.servo_pwm import MockPWM
from payload_catch.safety_runtime import SourceWatch
from tools.test_payload_servo import supervisor,authorized
from tools.test_safety_runtime import actual_class,Clock,Pub,N

class TimedReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.cfg=ServoConfig(enabled=True,calibrated=True,confirmation_mode='timed',delay_reference='operator_direct')
        self.path=Path(self.temp.name)/'attempt.json';self.backend=MockPWM()
        self.core=ServoInterlock(self.cfg,self.backend,AttemptRecord(self.path),mock=True)
        self.refresh(100,10)

    def refresh(self,wall,mono):
        self.core.on_supervisor(supervisor(wall),wall,mono);self.core.tick(wall,mono)

    def open(self):return self.core.authorize(authorized(planned_at=101.5),100,10)

    def test_evidence_mode_is_validated_and_changes_fleet_signature(self):
        self.assertNotEqual(servo_signature(self.cfg),servo_signature(replace(self.cfg,confirmation_mode='sensor',delay_reference='local_command_direct')))
        for value in ('automatic','',None,True):
            with self.assertRaises(ValueError):replace(self.cfg,confirmation_mode=value).validate()

    def test_no_sensor_required_but_complete_is_never_physical_release(self):
        self.assertEqual(self.core.state,'READY');self.assertTrue(self.open())
        self.assertEqual(self.backend.pulses,[1700])
        for index in range(1,30):
            self.refresh(100+index*.05,10+index*.05)
            self.assertEqual(self.core.state,'COMMAND_SENT')
        self.refresh(101.5,11.5)
        self.assertEqual(self.core.state,'ACTION_COMPLETE')
        self.assertFalse(self.core.physical_released);self.assertIsNone(self.core.detached_at)
        self.assertEqual(self.core.estimated_detach_at,101.5)
        self.core.abort();self.assertEqual(self.core.state,'ACTION_COMPLETE')
        self.assertFalse(self.core.authorize(authorized(102,planned_at=103.5),102,12))
        restarted=ServoInterlock(self.cfg,MockPWM(),AttemptRecord(self.path))
        self.assertEqual(restarted.state,'FAULT')

    def test_stop_before_during_and_permission_loss_do_not_complete(self):
        self.core.abort();self.assertFalse(self.open());self.assertFalse(self.backend.pulses)
        self.setUp();self.assertTrue(self.open());self.refresh(100.7,10.7)
        self.assertEqual(self.core.state,'COMMAND_SENT')
        self.core.abort();self.refresh(101.5,11.5)
        self.assertEqual(self.core.state,'FAULT');self.assertIsNone(self.core.action_completed_at)
        self.assertEqual(self.backend.pulses,[1700])
        self.setUp();self.assertTrue(self.open())
        state=supervisor(100.1);state['permits']['1']=False
        self.core.on_supervisor(state,100.1,10.1);self.core.tick(100.1,10.1)
        self.assertEqual(self.core.state,'FAULT')

    def test_missing_supervisor_and_late_completion_do_not_report_success(self):
        self.assertTrue(self.open());self.core.tick(100.21,10.21)
        self.assertEqual(self.core.state,'FAULT')
        self.setUp();self.assertTrue(self.open());self.refresh(101.6,11.6)
        self.assertEqual(self.core.state,'FAULT');self.assertIn('scheduler',self.core.reason)
        self.assertIsNone(self.core.action_completed_at)

    def test_real_write_receipt_defines_duration_and_missing_receipt_fails(self):
        self.core=ServoInterlock(self.cfg,self.backend,AttemptRecord(self.path),mock=False)
        self.refresh(100,10)
        self.backend.open=Mock(return_value=dict(write_started_at=100.005,write_completed_at=100.01))
        self.assertTrue(self.open());self.refresh(101.5,11.5)
        self.assertEqual(self.core.state,'COMMAND_SENT')
        self.refresh(101.511,11.511);self.assertEqual(self.core.state,'ACTION_COMPLETE')
        self.assertAlmostEqual(self.core.estimated_detach_at,101.51)
        self.setUp();self.core=ServoInterlock(self.cfg,self.backend,AttemptRecord(self.path),mock=False)
        self.refresh(100,10);self.assertFalse(self.open());self.assertEqual(self.core.state,'FAULT')

    def adapter(self):
        self.assertTrue(self.open());self.refresh(101.5,11.5)
        obj=actual_class('payload_servo_node.py',Clock(),dict(servo_signature=servo_signature))
        obj.core=self.core;obj.backend=self.backend;obj.output_closed=False;obj.action_reported=False
        obj.released_reported=False;obj.fault_reported=False;obj.release_id='code'
        for name in ('release_event_pub','action_complete_pub','released_pub','abort_pub','request_pub'):setattr(obj,name,Pub())
        return obj

    def test_completion_stops_pwm_first_and_never_publishes_physical_bool(self):
        obj=self.adapter();obj._sync();obj._sync()
        self.assertTrue(self.backend.closed);self.assertEqual(len(obj.release_event_pub.messages),1)
        self.assertEqual(len(obj.action_complete_pub.messages),1);self.assertFalse(obj.released_pub.messages)
        event=json.loads(obj.release_event_pub.messages[0].data)
        self.assertFalse(event['physically_confirmed']);self.assertIsNone(event['detached_at'])
        self.assertEqual(event['confirmation_mode'],'timed')

    def test_failed_stop_suppresses_completion_and_latches_fault(self):
        obj=self.adapter();self.backend.close=Mock(side_effect=OSError('disable failed'))
        obj._sync();self.assertFalse(obj.release_event_pub.messages);self.assertFalse(obj.action_complete_pub.messages)
        self.assertEqual(self.core.state,'FAULT');self.assertTrue(self.path.exists())

    def receiver(self):
        c=Clock();obj=actual_class('b_node.py',c)
        obj.release_confirmation_mode='timed';obj._supervisor_session='test'
        obj.expected_release_id='code';obj.expected_servo_signature='config'
        obj.clock_offset=0.;obj._release_event_nonce=None;obj._measured_release_at=None;obj._estimated_release_at=None
        obj.require_hardware_status=True;obj._dive_anchored=False;obj.released=False
        obj.mode='stack';obj.coord_mode='handshake';obj.phase='DIVE';obj.release_ref_t0=None
        event=dict(mock=False,session='test',nonce='once',release_id='code',servo_signature='config',
            confirmation_mode='timed',physically_confirmed=False,detached_at=None,
            stamp=c.time(),action_completed_at=c.time()-.01,estimated_detach_at=c.time()-.02,
            output_written_at=c.time()-1.52)
        return c,obj,event

    def test_b_distinguishes_estimated_time_rejects_wrong_mode_and_replay(self):
        c,obj,event=self.receiver()
        obj._on_released(N(data=True));self.assertFalse(obj.released)
        for changes in ({'confirmation_mode':'sensor'},{'physically_confirmed':True},{'detached_at':c.time()-.02},
                        {'stamp':c.time()-.3},{'mock':True},{'session':'old'},{'servo_signature':'old'}):
            obj._on_release_event(N(data=json.dumps(dict(event,**changes))));self.assertFalse(obj.released)
        obj._on_release_event(N(data=json.dumps(event)));self.assertTrue(obj.released)
        self.assertIsNone(obj._measured_release_at);self.assertEqual(obj._estimated_release_at,event['estimated_detach_at'])
        first=obj.release_ref_t0;event.update(nonce='new',estimated_detach_at=c.time()-.01)
        obj._on_release_event(N(data=json.dumps(event)));self.assertEqual(obj.release_ref_t0,first)
        c,obj,event=self.receiver();obj.release_confirmation_mode='sensor'
        obj._on_release_event(N(data=json.dumps(event)));self.assertFalse(obj.released)

    def test_a_requires_matching_timed_action_event_before_clearance(self):
        c=Clock();obj=actual_class('a_node.py',c)
        obj.release_confirmation_mode='timed';obj._supervisor_session='test'
        obj._released=True;obj._released_at=c.time();obj._release_cancelled=False
        obj._payload_released=False;obj._release_unconfirmed=False
        obj.expected_release_id='code';obj.expected_servo_signature='config'
        data=dict(mock=False,session='test',nonce='once',release_id='code',servo_signature='config',
            confirmation_mode='timed',physically_confirmed=False,detached_at=None,
            stamp=c.time(),action_completed_at=c.time()-.005,estimated_detach_at=c.time()-.01,planned_at=c.time())
        obj._on_released(N(data=True));self.assertFalse(obj._payload_released)
        for change in ({'physically_confirmed':True},{'mock':True},{'session':'old'},{'planned_at':c.time()+1}):
            obj._on_action_complete(N(data=json.dumps(dict(data,**change))))
            self.assertFalse(obj._payload_released)
        obj._on_action_complete(N(data=json.dumps(data)))
        self.assertTrue(obj._payload_released);self.assertTrue(obj._release_unconfirmed)

    def test_b_timed_hardware_health_does_not_claim_sensor_freshness(self):
        c,obj,event=self.receiver();obj._hardware_watch=SourceWatch(.2);obj._hardware_status_ready=False
        data=dict(stamp=c.time(),session='test',mock=False,ready=True,state='READY',
            confirmation_mode='timed',feedback_required=False,physically_released=False,
            release_id='code',servo_signature='config',feedback_stamp=None)
        obj._on_hardware_status(N(data=json.dumps(data)));self.assertTrue(obj._hardware_status_ready)
        data['confirmation_mode']='sensor';obj._on_hardware_status(N(data=json.dumps(data)))
        self.assertFalse(obj._hardware_watch.valid)

if __name__=='__main__':unittest.main()