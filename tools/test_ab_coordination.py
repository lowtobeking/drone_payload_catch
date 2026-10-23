#!/usr/bin/env python3
"""Fault injection on real A/B methods and analytic intercept geometry, no hardware."""
import json
import math
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.test_safety_runtime import actual_class, Clock, Pub, N, guard
from payload_catch.safety_runtime import SourceWatch
from payload_catch.stack_drop import plan_stack_drop, plan_stack_intercept, _stack_ref, _adaptive_dive


def receiver(c):
    o=actual_class('b_node.py',c,dict(plan_stack_intercept=plan_stack_intercept,
                                    _stack_ref=_stack_ref,_adaptive_dive=_adaptive_dive))
    o.clock_offset=.02;o.handshake_timeout=1.
    o._release_cmd=None;o._release_cmd_t=0.;o._release_cmd_stamp=-math.inf
    o._release_plan_at=None;o._release_cancelled=False;o.released=False
    o.holds=[];o.request_hold=o.holds.append
    return o


class CoordinationTests(unittest.TestCase):
    def test_bad_ack_never_becomes_fresh_on_receipt(self):
        for data in ([],[1],[1,2], [1,2,3,4], [math.nan,1,1],
                     [1,1012,1008], [1,1012,1011], [0,1012,1010],
                     [1,1017,1010], [1,1009,1010],[-1,1012,1010]):
            c=Clock();o=receiver(c);o._on_release_cmd(N(data=data))
            self.assertIsNone(o._release_cmd,data)
        c=Clock();o=receiver(c)
        o._on_release_cmd(N(data=[1,1011.7,1010.01]))
        self.assertIsNotNone(o._release_cmd)
        self.assertAlmostEqual(o._release_cmd_t,1009.99)

    def test_reordered_ack_and_pending_plan_replacement_are_rejected(self):
        c=Clock();o=receiver(c);o.clock_offset=0.
        o._on_release_cmd(N(data=[1,1011.7,1009.9]))
        first=o._release_cmd.copy()
        for data in ([1,1011.8,1009.8],[1,1011.8,1010.], [1,1011.7,1009.9]):
            o._on_release_cmd(N(data=data));self.assertEqual(o._release_cmd,first)
        o._on_release_cmd(N(data=[-1,0.,1010.]))
        self.assertTrue(o._release_cancelled);self.assertEqual(len(o.holds),1)
        c.mono+=.1;o._on_release_cmd(N(data=[1,1011.7,1010.1]))
        self.assertEqual(o._release_cmd[0],-1)

    def test_cancel_before_ack_and_after_physical_release(self):
        c=Clock();o=receiver(c);o.clock_offset=0.
        o._on_release_cmd(N(data=[-1,0.,1010.]))
        c.mono+=.1;o._on_release_cmd(N(data=[1,1011.7,1010.1]))
        self.assertTrue(o._release_cancelled);self.assertEqual(o._release_cmd[0],-1)
        o=receiver(c);o.released=True;o.release_ref_t0=1009.9
        for data in ([-1,0.,1010.1],[1,1011.7,1010.1]):
            o._on_release_cmd(N(data=data))
        self.assertFalse(o._release_cancelled);self.assertEqual(o.release_ref_t0,1009.9)
        self.assertFalse(o.holds)

    def test_a_ready_timeout_uses_source_time_and_rejects_invalid_metrics(self):
        c=Clock();o=actual_class('a_node.py',c)
        o._ready=None;o.ready_timeout=.5
        for data in ([1.,-.1,0.,c.time(),.01,.01,1.],
                     [1.,.01,0.,c.time(),-.01,.01,1.],
                     [1.,.01,0.,c.time(),.01,.01,2.]):
            o._on_b_ready(N(data=data));self.assertIsNone(o._ready)
        o._on_b_ready(N(data=[1.,.01,0.,c.time()-.4,.01,.01,1.]))
        self.assertAlmostEqual(o._ready_t,c.time()-.4)
        o.pub_pong=Pub()
        for data in ([],[1.,2.],[math.nan]):o._on_ping(N(data=data))
        self.assertFalse(o.pub_pong.messages)

    def test_a_cancel_then_flight_resume_cannot_recommit(self):
        c=Clock();o=actual_class('a_node.py',c)
        o.pub_release_abort=Pub();o.pub_release_cmd=Pub();o.permits=[]
        o.publish_release_permit=o.permits.append
        o._abort_release(c.time(),'test')
        o.on_operator_resume()
        o.mission_allowed=lambda: self.fail('cancelled release evaluated again')
        o._coord_tick(c.time()+1)
        self.assertEqual(o.permits,[False]);self.assertTrue(o._release_cancelled)

    def test_guard_cancel_is_terminal_even_with_fresh_new_schedule(self):
        c=Clock();o=guard(c);o._schedule(N(data=c.time()+.1));o._abort(N(data=True))
        c.mono+=.1
        o._permit(N(data=json.dumps(dict(stamp=c.time(),owner=0,allow=True))))
        o._schedule(N(data=c.time()+.1));o._tick()
        self.assertTrue(o.aborted);self.assertIsNone(o.release_at);self.assertFalse(o.pub.messages)

    def test_guard_cannot_move_pending_schedule_and_failed_execution_is_terminal(self):
        c=Clock();o=guard(c);t=c.time()+.1
        o._schedule(N(data=t));o._schedule(N(data=t+.1));self.assertEqual(o.release_at,t)
        o.supervisor['actions']['1']='hold';c.mono+=.1;o._tick()
        self.assertTrue(o.aborted);self.assertFalse(o.pub.messages)
        o._schedule(N(data=c.time()+.1));self.assertIsNone(o.release_at)

    def test_current_state_plan_matches_contact_and_braking_geometry(self):
        p=plan_stack_intercept(4.,3.,1.2,.3,a_dive=3.,funnel_depth=2.)
        self.assertTrue(p.feasible,p.reason)
        pp=-4+1.2*p.t_c+.5*9.81*p.t_c**2
        pr,vr,_=_stack_ref(p.t_c,p,(0,0),9.81)
        self.assertAlmostEqual(pp,pr[2]);self.assertAlmostEqual(vr[2],p.v_b)
        self.assertAlmostEqual(p.v_pay-p.v_b,p.v_rel)
        end=p.t_c+p.v_b/p.a_brake
        pe,ve,_=_stack_ref(end+1e-6,p,(0,0),9.81)
        self.assertAlmostEqual(-pe[2],p.post_brake_alt);self.assertEqual(ve[2],0.)
        _,v0,_=_stack_ref(0,p,(0,0),9.81)
        self.assertEqual(v0[2],.3)

    def test_delay_consumes_real_fall_distance_and_changes_remaining_time(self):
        initial=plan_stack_drop(4.2,3.,a_dive=3.,funnel_depth=2.)
        delay=.15
        current=plan_stack_intercept(4.2-.5*9.81*delay**2,3.,9.81*delay,0.,
                                     a_dive=3.,funnel_depth=2.)
        wrong=plan_stack_drop(4.2,3.,a_dive=3.,funnel_depth=2.)
        self.assertTrue(current.feasible,current.reason)
        self.assertLess(current.t_c,wrong.t_c)
        self.assertGreater(current.v_rel,initial.v_rel)

    def test_intercept_rejects_missed_payload_speed_and_ground_risk(self):
        cases=[dict(payload_height=2.,b_height=3.,payload_vz=1.,b_vz=0.),
               dict(payload_height=4.,b_height=3.,payload_vz=8.,b_vz=0.),
               dict(payload_height=1.5,b_height=1.,payload_vz=3.,b_vz=3.),
               dict(payload_height=4.,b_height=3.,payload_vz=math.nan,b_vz=0.),
               dict(payload_height=4.,b_height=3.,payload_vz=1.,b_vz=0.,a_brake=0.)]
        for case in cases:
            p=plan_stack_intercept(**case)
            self.assertFalse(p.feasible,case)

    def test_zero_initial_velocity_preserves_nominal_planning(self):
        for gap in (.1,.5,1.,2.):
            a=plan_stack_drop(5+gap,5.,a_dive=None)
            b=plan_stack_intercept(5+gap,5.,0.,0.,a_dive=None)
            self.assertEqual(a.feasible,b.feasible)
            for name in ('a_dive','t_c','v_rel','post_brake_alt'):
                self.assertAlmostEqual(getattr(a,name),getattr(b,name))

    def test_payload_source_age_and_pre_detachment_sample(self):
        c=Clock();o=receiver(c);o._measured_release_at=c.time()-.1
        o._payload_watch=SourceWatch();o.t_sim=c.time()-.05
        o.p_pay=np.array([0.,0.,-4.]);o.v_pay=np.array([.2,0.,1.])
        o._payload_watch.accept(o.t_sim,[],c.time(),c.mono)
        p,v=o._payload_for_intercept(c.time())
        self.assertAlmostEqual(p[2],-4+.05+.5*9.81*.05**2)
        self.assertAlmostEqual(v[2],1+9.81*.05)
        o.t_sim=c.time()-.2;self.assertIsNone(o._payload_for_intercept(c.time()))

    def test_real_control_preserves_physical_edge_and_uses_current_state(self):
        c=Clock();o=receiver(c);o.clock_offset=0.
        o.released=True;o.phase='DIVE';o.coord_mode='handshake'
        o.pos_world=np.array([0.,0.,-3.]);o.vel=np.array([0.,0.,.2])
        o.a_est=np.array([0.,0.,-4.2]);o.a_release=o.a_est.copy();o.standby=o.pos_world.copy()
        o._relnav_a=lambda now:(None,None);o.caught=False;o._dive_anchored=False
        o._measured_release_at=c.time()-.15;o.release_ref_t0=o._measured_release_at
        o._payload_watch=SourceWatch();o.t_sim=c.time()-.02
        o._payload_watch.accept(o.t_sim,[],c.time(),c.mono)
        o.p_pay=np.array([0.,0.,-4.]);o.v_pay=np.array([0.,0.,1.])
        o.dive_anchor_vz=.1;o.require_physical_capture=True;o.auto_min_dive=False
        o.funnel_mount_height=.1;o.a_dive=3.;o.a_brake=6.;o.funnel_depth=2.
        o.funnel_restitution=.6;o.adaptive_alt_floor=.3;o.adaptive_dive=False
        o.get_parameter=lambda name:N(value=6.)
        o.use_formation=False;o.track_payload=False;o.stack_kp_xy=1.;o.stack_kp_z=1.
        o.zem_gain=0.;o.v_max=3.;o.a_ff_gain=1.;o.yaw=0.;o.pub_miss=Pub()
        o._stack_capture_check=lambda:None;o.velocities=[]
        o.publish_velocity=lambda v,**kwargs:o.velocities.append(v.copy())
        o.control_stack(c.time())
        self.assertFalse(o.holds);self.assertTrue(o._dive_anchored)
        self.assertAlmostEqual(o.release_ref_t0,c.time()-.15)
        self.assertAlmostEqual(o._dive_t0,c.time())
        self.assertAlmostEqual(o.stack_plan.v_b0,.2)
        self.assertAlmostEqual(o.stack_plan.gap,4-.02-.5*9.81*.02**2-.1-3)
        self.assertAlmostEqual(o.velocities[-1][2],.2)


if __name__=='__main__':unittest.main()
