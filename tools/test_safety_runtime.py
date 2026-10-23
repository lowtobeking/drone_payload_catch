#!/usr/bin/env python3
"""Offline fault injection on actual node methods; no ROS/PX4 control traffic."""
import ast
import __future__
import json
import math
import sys
import time
import types
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from payload_catch.safety_runtime import (SourceWatch,CommandTicket,landing_order,
    final_setpoint,flight_reasons,LEASE_S,FAULT_LAND_S)
from payload_catch.preflight_health import approval_reason
from payload_catch.relnav import geodetic_to_ned,compensate_lever_arm,a_state_for_b,VelEstimator
from tools.test_live_probe import healthy
N=types.SimpleNamespace

class Pub:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)

class Clock:
    mono=10.
    def monotonic(self):return self.mono
    def time(self):return 1000+self.mono

def actual_class(file,clock,extra=None):
    path=ROOT/'payload_catch'/file
    node=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
    node.bases=[];node.decorator_list=[]
    node.body=[n for n in node.body if getattr(n,'name','')!='__init__']
    ns=dict(time=clock,json=json,uuid=uuid,math=math,np=np,Bool=N,String=N,Float64=N,
        Float64MultiArray=N,VehicleCommand=N,TrajectorySetpoint=N,
        SourceWatch=SourceWatch,CommandTicket=CommandTicket,flight_reasons=flight_reasons,
        final_setpoint=final_setpoint,landing_order=landing_order,LEASE_S=LEASE_S,
        FAULT_LAND_S=FAULT_LAND_S,geodetic_to_ned=geodetic_to_ned,
        compensate_lever_arm=compensate_lever_arm,a_state_for_b=a_state_for_b)
    ns.update(extra or {})
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec',
        flags=__future__.annotations.compiler_flag),ns)
    obj=ns[node.name]()
    obj.get_clock=lambda:N(now=lambda:N(nanoseconds=int(clock.time()*1e9)))
    obj.get_logger=lambda:N(error=lambda *a:None,warn=lambda *a:None,info=lambda *a:None)
    return obj

def supervisor(clock):
    o=actual_class('safety_supervisor.py',clock)
    o.require_rc=False;o.snaps={i:healthy() for i in (0,1)}
    o.health={i:N(snapshot=lambda now,i=i:o.snaps[i]) for i in (0,1)}
    o.status={i:(clock.mono,N(arming_state=2,nav_state=14)) for i in (0,1)}
    o.position={};o.landed={};o.task={i:dict(ever_offboard=True,safety='OK',sources_ready=True) for i in (0,1)}
    o.task_watch={i:SourceWatch() for i in (0,1)}
    for w in o.task_watch.values():w.accept(clock.time(),[],clock.time(),clock.mono)
    o.action={i:'run' for i in (0,1)};o.reason={i:'' for i in (0,1)}
    o.resume_token={i:'' for i in (0,1)};o.resume_until={};o.fault_since={};o.tickets={};o.ack_stamp={}
    o.land_targets=set();o.released=False;o.session='test';o.seq=0
    o.command_pub={i:Pub() for i in (0,1)};o.state_pub=Pub();o.abort_pub=Pub()
    return o

def guard(clock):
    o=actual_class('release_guard.py',clock)
    o.aborted=False;o.release_delay_s=0.;o.owner=0;o.permit_watch=SourceWatch(.2);o.supervisor_watch=SourceWatch()
    o.allow=False;o.supervisor=None;o.release_at=None;o.released=False;o.last_schedule=-math.inf;o.pub=Pub()
    o._permit(N(data=json.dumps(dict(stamp=clock.time(),owner=0,allow=True))))
    o._supervisor(N(data=json.dumps(dict(stamp=clock.time(),session='test',
        permits={'0':True,'1':True},actions={'0':'run','1':'run'},release_permitted=True))))
    return o

class RuntimeTests(unittest.TestCase):
    def test_source_duplicate_replay_nan_and_missing(self):
        w=SourceWatch();self.assertFalse(w.fresh(10,1))
        self.assertTrue(w.accept(10,[1],10,1))
        self.assertFalse(w.accept(10,[1],10.4,1.4))
        self.assertFalse(w.accept(9,[1],10.4,1.4))
        self.assertFalse(w.fresh(10.6,1.6))
        self.assertFalse(w.accept(10.7,[math.nan],10.7,1.7))
        self.assertFalse(w.accept(12,[1],11,2))

    def test_final_published_messages_override_feedforward(self):
        c=Clock();o=actual_class('px4_iface.py',c)
        o._operator_paused=o._killed=o._landing=False;o._safety_state='HOLD'
        o.sp_rate_limit=0;o.sensor_constraints_enable=False;o._last_vsp=o._prev_vsp=None;o.pub_sp=Pub()
        o.publish_velocity([4,5,6],acc_ff=[7,8,9]);m=o.pub_sp.messages[-1]
        self.assertEqual(m.velocity,[0,0,0]);self.assertTrue(all(math.isnan(v) for v in m.acceleration))
        o.publish_position([100,100,-100],vel_ff=[5,5,5]);m=o.pub_sp.messages[-1]
        self.assertTrue(all(math.isnan(v) for v in m.position));self.assertEqual(m.velocity,[0,0,0])
        o._safety_state='PULLBACK';seen=[]
        o._fence_velocity=lambda v:seen.append(v) or [-1,0,0]
        o.publish_velocity([99,99,99],acc_ff=[99,99,99])
        self.assertEqual(seen,[[0,0,0]]);self.assertEqual(o.pub_sp.messages[-1].velocity,[-1,0,0])
        o._operator_paused=True;n=len(o.pub_sp.messages);o.publish_velocity([1,1,1])
        self.assertEqual(len(o.pub_sp.messages),n)

    def test_battery_40_only_before_takeoff_and_rc_receipt(self):
        s=healthy();s.batt_remaining_min=.39
        self.assertEqual(flight_reasons(s),[])
        s.batt_remaining_min=.06;self.assertIn('battery_critical',flight_reasons(s))
        s=healthy();s.ages['rc']=.51
        self.assertIn('rc_missing',flight_reasons(s,require_rc=True))
        receipt=dict(schema=1,passed=True,checked_at=100,battery_min=.4,drone_ids=[0,1],gps=True,params_checked=[0,1])
        self.assertTrue(approval_reason(receipt,now=100,require_rc=True))
        self.assertFalse(approval_reason({**receipt,'rc':True},now=100,require_rc=True))

    def test_pause_latched_and_pair_resume_token(self):
        c=Clock();o=supervisor(c);o._operator('pause',(0,1));o._tick()
        self.assertEqual(o.action,{0:'hold',1:'hold'});self.assertTrue(o.abort_pub.messages)
        o._tick();self.assertEqual(o.action[0],'hold')
        o.task[0]['safety']='HOLD';o.task[1]['safety']='HOLD'
        o._operator('resume',(0,));o._tick()
        d=json.loads(o.state_pub.messages[-1].data)
        self.assertEqual(o.action,{0:'run',1:'run'});self.assertTrue(all(d['permits'].values()))
        self.assertTrue(all(o.resume_token.values()))
        o.released=True;o._operator('pause',(0,1));o._operator('resume',(0,1))
        self.assertEqual(o.action[0],'hold')

    def test_task_pause_idempotent_explicit_resume_and_supervisor_restart(self):
        c=Clock();o=actual_class('px4_iface.py',c)
        hooks=[];o._operator_paused=False;o._supervisor_permit=True;o._release_abort_pub=Pub()
        o.on_safety_stop=lambda reason:hooks.append('stop');o.on_operator_resume=lambda:hooks.append('resume')
        o._supervisor_watch=SourceWatch();o._supervisor_session='first';o._ever_offboard=True
        o._resume_token='';o.drone_id=0;o._killed=o._landing=False
        o._safety_request=lambda *a:None
        def state(session='first',token=''):
            return N(data=json.dumps(dict(stamp=c.time(),session=session,actions={'0':'run'},
                resume_tokens={'0':token},permits={'0':True},reasons={'0':''})))
        with patch.dict(sys.modules,{'std_msgs':N(msg=N(Bool=N)),'std_msgs.msg':N(Bool=N)}):
            o._pause_task('stop');o._pause_task('repeat')
            self.assertEqual(hooks,['stop']);self.assertFalse(o._supervisor_permit)
            o._on_supervisor(state());self.assertTrue(o._operator_paused)
            c.mono+=.01;o._on_supervisor(state(token='explicit'))
            self.assertFalse(o._operator_paused);self.assertEqual(hooks,['stop','resume'])
            c.mono+=.01;o._on_supervisor(state(session='restarted',token='new'))
            self.assertTrue(o._operator_paused);self.assertFalse(o._supervisor_permit)

    def test_infeasible_capture_plan_revokes_ready_and_direct_permit(self):
        c=Clock();o=actual_class('b_node.py',c)
        o._plan_before_release=lambda:N(feasible=False);o.mission_allowed=lambda:True
        o.coord_mode='direct';events=[];o.publish_release_permit=lambda allow:events.append(allow)
        o._sigma_est=.01;o.rel_sigma_floor=.01;o.sigma_model='absolute'
        o.sensor_constraints_enable=False;o.pub_ready=Pub()
        o._pub_ready(c.time(),True,0,0)
        self.assertEqual(o.pub_ready.messages[-1].data[0],0)
        self.assertEqual(o.pub_ready.messages[-1].data[6],0)
        self.assertEqual(events,[False])

    def test_simulator_release_cannot_execute_pre_cancel_envelope(self):
        c=Clock();o=actual_class('payload_node.py',c)
        o._authorized=False;o._authorized_watch=SourceWatch(.1);o._abort_at=-math.inf
        o.model=N(released=False);o.abs_release_at=None
        stamp=c.time();o._on_release_abort(N(data=True))
        o._on_authorized(N(data=json.dumps(dict(stamp=stamp,nonce='old'))))
        self.assertFalse(o._authorized)
        c.mono+=.01;o._on_authorized(N(data=json.dumps(dict(stamp=c.time(),nonce='fresh'))))
        self.assertTrue(o._authorized)

    def test_task_pause_before_health_fault_still_escalates(self):
        c=Clock();o=supervisor(c);o._operator('pause',(0,),'task fault')
        o.snaps[0].xy_valid_frac=0;o._tick();self.assertIn(0,o.fault_since)
        c.mono+=3.1;o._tick()
        self.assertEqual(o.action,{0:'hold',1:'land'})
        o.status[1]=(c.mono,N(arming_state=1,nav_state=18));o.landed[1]=(c.mono,True);o._tick()
        self.assertEqual(o.action[0],'land')

    def test_hung_task_independent_hold_and_bounded_commands(self):
        c=Clock();o=supervisor(c);c.mono+=.6;o._tick()
        self.assertEqual(o.action,{0:'hold',1:'hold'})
        self.assertTrue(o.command_pub[0].messages)
        for step in range(30):c.mono+=.31;o._tick()
        self.assertLessEqual(len(o.command_pub[0].messages),8)
        self.assertEqual(o.tickets[0].failure,'EXECUTION_UNCONFIRMED')

    def test_manual_and_px4_landing_never_overridden(self):
        for nav in (0,1,2,18,13):
            c=Clock();o=supervisor(c);o.status[0]=(c.mono,N(arming_state=2,nav_state=nav));o._tick()
            self.assertEqual(o.action[0],'manual',nav)
            self.assertFalse(o.command_pub[0].messages,nav)
            o._operator('pause',(0,1));o._tick();self.assertFalse(o.command_pub[0].messages)
            o._operator('resume',(0,1));self.assertEqual(o.action[0],'manual')

    def test_pair_safety_and_both_offboard_needed_for_release(self):
        c=Clock();o=supervisor(c);o.task[1]['safety']='HOLD';o.task[1]['ever_offboard']=False;o._tick()
        self.assertFalse(any(json.loads(o.state_pub.messages[-1].data)['permits'].values()))
        o=supervisor(c);o.task[1]['ever_offboard']=False;o.status[1]=(c.mono,N(arming_state=1,nav_state=0));o._tick()
        self.assertFalse(json.loads(o.state_pub.messages[-1].data)['release_permitted'])

    def test_landing_never_uses_independent_local_coordinates(self):
        self.assertEqual(landing_order({0},{0:[0,0,-4],1:[100,0,-3]},{1:False},{}),{0:'hold',1:'land'})
        self.assertEqual(landing_order({0},{},{},{1:True}),{0:'land'})

    def test_ack_filter_denial_retries_and_mode_confirmation(self):
        t=CommandTicket('kill',185,13,0)
        self.assertFalse(t.acknowledge(185,0,1,1));self.assertFalse(t.acknowledge(21,0,255,191))
        self.assertEqual(t.update(0,status_fresh=True,nav_state=13,armed=True),'PENDING')
        t.acknowledge(185,2,255,191);self.assertIn('ACK_REJECTED',t.update(0,status_fresh=True,nav_state=13,armed=True))
        self.assertFalse(t.due(1))
        t=CommandTicket('land',21,18,0);t.acknowledge(21,0,255,191)
        self.assertEqual(t.update(0,status_fresh=False,nav_state=18,armed=True),'PENDING')
        self.assertEqual(t.update(1,status_fresh=True,nav_state=18,armed=True),'MODE_CONFIRMED')
        self.assertEqual(t.update(2,status_fresh=True,nav_state=18,armed=False,landed=True),'LANDED_CONFIRMED')

    def test_release_guard_expired_revoked_late_cancel_and_one_shot(self):
        for case in ('permit','supervisor','peer','not_armed','late','cancel'):
            c=Clock();o=guard(c);o._schedule(N(data=c.time()+.1))
            if case=='permit':o.permit_watch.valid=False
            if case=='supervisor':o.supervisor_watch.valid=False
            if case=='peer':o.supervisor['permits']['1']=False
            if case=='not_armed':o.supervisor['release_permitted']=False
            if case=='cancel':
                o._abort(N(data=True));o._schedule(N(data=c.time()+.1))
                self.assertIsNone(o.release_at)
            c.mono+=.25 if case=='late' else .1;o._tick();self.assertFalse(o.pub.messages,case)
        c=Clock();o=guard(c);o._schedule(N(data=c.time()+.1));c.mono+=.1;o._tick();o._tick()
        self.assertEqual(len(o.pub.messages),1)

    def test_guard_compensates_measured_delay_and_still_cancels(self):
        c=Clock();o=guard(c);o.release_delay_s=.08
        planned=c.time()+.2;o._schedule(N(data=planned))
        c.mono+=.1;o._tick();self.assertFalse(o.pub.messages)
        c.mono+=.021;o._tick();self.assertEqual(len(o.pub.messages),1)
        envelope=json.loads(o.pub.messages[0].data)
        self.assertAlmostEqual(envelope['planned_at'],planned)
        self.assertAlmostEqual(envelope['command_at'],planned-.08)
        self.assertEqual(envelope['session'],'test')
        c=Clock();o=guard(c);o.release_delay_s=.08
        o._schedule(N(data=c.time()+.04));self.assertIsNone(o.release_at)
        o._schedule(N(data=c.time()+.2));o._abort(N(data=True))
        c.mono+=.13;o._tick();self.assertFalse(o.pub.messages)


    def test_guard_schedules_one_point_five_seconds_before_detachment(self):
        c=Clock();o=guard(c);o.release_delay_s=1.5
        start=c.time();planned=start+1.7
        o._schedule(N(data=planned))
        c.mono+=.19;o._tick();self.assertFalse(o.pub.messages)
        c.mono+=.011
        o._permit(N(data=json.dumps(dict(stamp=c.time(),owner=0,allow=True))))
        o._supervisor(N(data=json.dumps(dict(stamp=c.time(),session='test',
            permits={'0':True,'1':True},actions={'0':'run','1':'run'},release_permitted=True))))
        o._tick();self.assertEqual(len(o.pub.messages),1)
        envelope=json.loads(o.pub.messages[0].data)
        self.assertAlmostEqual(envelope['command_at'],start+.2)
        self.assertAlmostEqual(envelope['planned_at'],planned)
        self.assertAlmostEqual(envelope['planned_at']-envelope['command_at'],1.5)
        c.mono+=1.5;o._tick();self.assertEqual(len(o.pub.messages),1)

    def test_b_tracks_formation_without_diving_during_long_opening(self):
        c=Clock();o=actual_class('b_node.py',c)
        o.pos_world=np.array([0.,0.,-3.]);o.a_est=np.array([0.,0.,-4.])
        o.a_release=o.a_est.copy();o.standby=o.pos_world.copy()
        o._release_cancelled=False
        o.coord_mode='handshake';o.phase='DIVE';o._release_cmd=None
        o._relnav_a=lambda now:(None,None)
        o.released=False;o.caught=False;o._dive_anchored=False
        o.use_formation=True;o.formation_vel=np.array([1.,0.,0.]);o.vel=o.formation_vel.copy()
        o.align_xy_tol=.1;o.align_vel_tol=.1;o.form_kp_rel=1.;o.v_max=2.;o.yaw=0.
        o.release_ref_t0=c.time()+1.7
        o.hover_velocity=lambda *args,**kwargs:np.zeros(3)
        ready=[];velocities=[];holds=[]
        o._pub_ready=lambda now,ok,xy,speed:ready.append(ok)
        o.publish_velocity=lambda v,**kwargs:velocities.append(v.copy())
        o.request_hold=holds.append
        for elapsed in (.2,.8,1.49,1.69,1.75):
            c.mono=10+elapsed
            o.control_stack(c.time())
            np.testing.assert_allclose(velocities[-1],[1.,0.,0.])
            self.assertFalse(o._dive_anchored)
        self.assertTrue(all(ready));self.assertFalse(holds)
        c.mono=12.21;o.control_stack(c.time())
        self.assertEqual(holds,['release execution unconfirmed'])


    def test_b_uses_physical_edge_not_dds_receive_or_planned_time(self):
        c=Clock();o=actual_class('b_node.py',c)
        o._supervisor_session='test';o.expected_release_id='r';o.expected_servo_signature='s'
        o.clock_offset=.01;o._release_event_nonce=None;o._measured_release_at=None
        o.require_hardware_status=True;o._dive_anchored=False
        o.released=False;o.mode='stack';o.coord_mode='handshake';o.phase='DIVE'
        o.release_ref_t0=c.time()+.05
        o._on_released(N(data=True));self.assertFalse(o.released)
        d=dict(mock=False,session='test',nonce='n',release_id='r',servo_signature='s',
               stamp=c.time()+.005,detached_at=c.time()-.06)
        for change in ({'mock':True},{'session':'old'},{'servo_signature':'old'},
                       {'detached_at':c.time()+1},{'stamp':c.time()-.5}):
            o._on_release_event(N(data=json.dumps(dict(d,**change))))
            self.assertFalse(o.released)
        o._on_release_event(N(data=json.dumps(d)))
        self.assertTrue(o.released)
        self.assertAlmostEqual(o.release_ref_t0,c.time()-.07)
        first=o.release_ref_t0
        o._on_release_event(N(data=json.dumps(dict(d,nonce='new',detached_at=c.time()-.01))))
        self.assertEqual(o.release_ref_t0,first)

    def test_real_hardware_status_rejects_mock_stale_and_other_session(self):
        c=Clock();o=actual_class('b_node.py',c)
        o._supervisor_session='test';o._hardware_watch=SourceWatch(.2)
        o._hardware_status_ready=False
        def status(**changes):
            d=dict(stamp=c.time(),feedback_stamp=c.time(),session='test',mock=False,
                   ready=True,state='READY');d.update(changes)
            return N(data=json.dumps(d))
        o._on_hardware_status(status(mock=True));self.assertFalse(o._hardware_watch.valid)
        o._on_hardware_status(status(session='other'));self.assertFalse(o._hardware_watch.valid)
        o._on_hardware_status(status(feedback_stamp=c.time()-.3));self.assertFalse(o._hardware_watch.valid)
        o._on_hardware_status(status());self.assertTrue(o._hardware_status_ready)
        self.assertTrue(o._hardware_watch.fresh(c.time(),c.mono))
        c.mono+=.21;self.assertFalse(o._hardware_watch.fresh(c.time(),c.mono))

    def test_servo_rollback_failure_remains_visible_after_operator_fault(self):
        c=Clock();o=actual_class('payload_servo_node.py',c)
        o.output_closed=False
        def fail():raise OSError('disable failed')
        o.backend=N(close=fail)
        o.core=N(reason='operator pause',fault=lambda reason:None)
        o._close_output()
        self.assertTrue(o.output_closed)
        self.assertIn('operator pause',o.core.reason)
        self.assertIn('PWM rollback needs inspection',o.core.reason)

    def test_fleet_rejects_old_servo_code_and_other_calibration(self):
        c=Clock();o=actual_class('b_node.py',c)
        o._supervisor_session='test';o._hardware_watch=SourceWatch(.2)
        o._hardware_status_ready=False;o.expected_release_id='new-code';o.expected_servo_signature='measured-config'
        d=dict(stamp=c.time(),feedback_stamp=c.time(),session='test',mock=False,ready=True,state='READY',
               release_id='old-code',servo_signature='measured-config')
        o._on_hardware_status(N(data=json.dumps(d)));self.assertFalse(o._hardware_watch.valid)
        d.update(release_id='new-code',servo_signature='other-calibration')
        o._on_hardware_status(N(data=json.dumps(d)));self.assertFalse(o._hardware_watch.valid)
        d.update(servo_signature='measured-config')
        o._on_hardware_status(N(data=json.dumps(d)));self.assertTrue(o._hardware_status_ready)

    def test_fleet_lease_blocks_takeoff_release_and_reports_not_ready(self):
        c=Clock();o=actual_class('px4_iface.py',c)
        o.require_fleet_link=True;o._fleet_watch=SourceWatch(.2);o._fleet_ok=False
        self.assertFalse(o._fleet_ready())
        self.assertFalse(o._takeoff_ready())
        self.assertFalse(o.mission_allowed())
        o._on_fleet_link(N(data=json.dumps(dict(stamp=c.time(),ready=True))))
        self.assertTrue(o._fleet_ready())
        c.mono+=.21;self.assertFalse(o._fleet_ready())
        o._on_fleet_link(N(data=json.dumps(dict(stamp=c.time(),ready=False))))
        self.assertFalse(o._fleet_ready())

    def test_payload_capture_missing_contact_present_or_stale(self):
        c=Clock();o=actual_class('b_node.py',c)
        o.caught=False;o.released=True;o.p_pay=np.array([0.,0.,-3.21]);o.v_pay=np.zeros(3)
        o.pos_world=np.array([0.,0.,-3.]);o.vel=np.zeros(3);o.mission_allowed=lambda:True
        o._payload_watch=SourceWatch();o._payload_watch.accept(c.time(),[],c.time(),c.mono)
        o.px4_z_bias=0;o.funnel_mount_height=.21;o.catch_z_tol=.1;o.funnel_eff_radius=.12;o.v_retain=1
        o.contact_detect=True;o.require_physical_capture=True;o._contact_last_t=c.time();o._contact_last_vz=0
        o._near_since=None;o._ext_contact=False;o._payload_present=False;o._payload_present_t=c.mono
        o.contact=N(update=lambda *a,**kw:False);o._capture_since=None;o.pub_caught=Pub();o._lock_req_pub=None
        for step in range(12):c.mono+=.01;o._stack_capture_check()
        self.assertFalse(o.caught) # no timeout fallback
        o.contact=N(update=lambda *a,**kw:True);o._stack_capture_check();self.assertFalse(o.caught)
        o._payload_present=True;o._payload_present_t=c.mono;o._stack_capture_check();self.assertFalse(o.caught)
        c.mono+=.11;o._stack_capture_check();self.assertFalse(o.caught) # cached geometry isn't retention
        o._payload_watch.accept(c.time(),[],c.time(),c.mono)
        o._stack_capture_check();self.assertTrue(o.caught)
        o.caught=False;c.mono+=1;o._capture_check();self.assertFalse(o.caught)

    def test_rtk_source_covariance_sync_frame_and_no_cached_retimestamp(self):
        c=Clock();o=actual_class('relnav_node.py',c)
        o.rtk_type='navsatfix';o.sigma_max=.03;o.sync=.10;o.source='rtk';o.use_geo=False;o.origin=None
        o.watch={k:SourceWatch() for k in ('a','b','b_pos','a_att','b_att')}
        o.fix={};o.sigma={};o.pose={'b':np.array([5.,0.,-3.])};o.att={'a':(0,0,0),'b':(0,0,0)}
        o.a_lever=np.array([0,0,.2]);o.b_lever=np.array([0,0,.1]);o.last_source=None
        o.vel=VelEstimator();o.pub=Pub();o.valid_pub=Pub()
        def fix(x,z,variance=.0001,stamp=None,covtype=2):
            t=c.time() if stamp is None else stamp
            return N(header=N(stamp=N(sec=int(t),nanosec=int(round((t-int(t))*1e9)))),
                latitude=x,longitude=0,altitude=z,status=N(status=0),
                position_covariance=[variance,0,0,0,variance,0,0,0,variance],position_covariance_type=covtype)
        o._on_fix(fix(0,-4), 'a');o._on_fix(fix(0,-3), 'b')
        for key in ('b_pos','a_att','b_att'):o.watch[key].accept(c.time(),[],c.time(),c.mono)
        o._tick();self.assertTrue(o.valid_pub.messages[-1].data)
        self.assertTrue(np.allclose(o.pub.messages[-1].data[1:4],[5,0,-3.9]))
        o._on_fix(fix(0,-4),'a');o._tick();self.assertEqual(len(o.pub.messages),1)
        c.mono+=.6;o._tick();self.assertFalse(o.valid_pub.messages[-1].data)
        o._on_fix(fix(0,-4,covtype=0),'a');self.assertFalse(o.watch['a'].valid)
        o._on_fix(fix(0,-4,variance=.001),'a');self.assertFalse(o.watch['a'].valid)
        o.rtk_type='array';o._on_fix(N(data=[0,0,-4,.01]),'a');self.assertFalse(o.watch['a'].valid)

if __name__=='__main__':unittest.main()
