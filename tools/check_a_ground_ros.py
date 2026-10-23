#!/usr/bin/env python3
"""A coordination -> guard -> mock PWM rehearsal in an isolated network namespace.

This TEST replaces A's mission permission with a fixture and cancels its flight
timers. It cannot validate preflight sensors, flight, or real latch calibration.
No PX4 commands or real PWM backend are allowed.
"""
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def scenario(mode):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from std_msgs.msg import Bool,Float64,Float64MultiArray,String
    from px4_msgs.msg import VehicleCommand,OffboardControlMode,TrajectorySetpoint
    from payload_catch.a_node import ANode
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo
    with tempfile.TemporaryDirectory() as temporary:
        rclpy.init(args=['--ros-args','-p','auto_arm:=false','-p','coord_mode:=handshake',
            '-p','preflight_report:=/nonexistent/ground-rehearsal-no-flight.json',
            '-p','release_lead:=1.7','-p','release_delay_s:=1.5',
            '-p','backend:=mock','-p','enabled:=true','-p','calibrated:=true',
            '-p','attempt_record:='+str(Path(temporary)/'attempt.json')])
        a=ANode()
        for timer in tuple(a.timers):timer.cancel()
        assert not a.auto_arm and not a.mission_allowed(), 'real flight gate must remain closed'
        # Only this test process overrides the mission gate, inside separate netns.
        a.mission_allowed=lambda:True
        a._pos=a.hover-a.world_offset;a._vel[:]=0.
        guard=ReleaseGuard();servo=PayloadServo();bench=Node('a_ground_rehearsal')
        assert servo.core.mock and type(servo.backend).__name__=='MockPWM'
        executor=SingleThreadedExecutor();nodes=[a,guard,servo,bench]
        for node in nodes:executor.add_node(node)
        pubs={name:bench.create_publisher(kind,name,10) for kind,name in (
            (String,'/safety/state'),(Float64MultiArray,'/drone_b/ready'),
            (String,'/payload/mock/latch_feedback'),(Float64,'/payload/release_at'))}
        received=[];events=[];hardware_ready=[];counts=dict(command=0,mode=0,setpoint=0)
        bench.create_subscription(String,'/payload/authorized_release',lambda m:received.append(json.loads(m.data)),10)
        bench.create_subscription(String,'/payload/mock/release_event',lambda m:events.append(json.loads(m.data)),10)
        bench.create_subscription(Bool,'/payload/hardware_ready',lambda m:hardware_ready.append(m.data),10)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        def monitor(key):
            def cb(msg):counts[key]+=1
            return cb
        for kind,topic,key in ((VehicleCommand,'/fmu/in/vehicle_command','command'),
                (OffboardControlMode,'/fmu/in/offboard_control_mode','mode'),
                (TrajectorySetpoint,'/fmu/in/trajectory_setpoint','setpoint')):
            bench.create_subscription(kind,topic,monitor(key),qos)
        ready=False;detach=False
        def tick():
            stamp=time.time()
            pubs['/safety/state'].publish(String(data=json.dumps(dict(stamp=stamp,session='ground-fixture',
                permits={'0':True,'1':True},actions={'0':'run','1':'run'},
                reasons={'0':'','1':''},resume_tokens={'0':'','1':''},release_permitted=True))))
            if ready:
                pubs['/drone_b/ready'].publish(Float64MultiArray(data=[1.,.01,.01,stamp,.01,.01,1.]))
            data=dict(stamp=stamp,session='ground-fixture',locked=not detach,
                      present=not detach,detached=detach,healthy=True)
            if detach:data['detached_at']=servo.core.planned_at
            pubs['/payload/mock/latch_feedback'].publish(String(data=json.dumps(data)))
            a._coord_tick(a.get_clock().now().nanoseconds*1e-9)
        timer=bench.create_timer(.02,tick)
        def until(predicate,timeout=3.):
            deadline=time.monotonic()+timeout
            while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.003)
            assert predicate(),(mode,servo.core.state,servo.core.reason,a._release_cancelled)
        def spin(seconds):
            deadline=time.monotonic()+seconds
            until(lambda:time.monotonic()>=deadline,seconds+1)
        try:
            until(lambda:servo.core.state=='READY')
            spin(.25)
            assert not a._released and not servo.backend.pulses,'missing B ready must block A'
            ready=True
            until(lambda:a._released)
            planned=a._released_at
            if mode=='normal':
                until(lambda:servo.core.state=='COMMAND_SENT')
                assert servo.backend.pulses==[1700] and not servo.core.physical_released
                until(lambda:time.time()>=planned-.10)
                assert not servo.core.physical_released and not events
                until(lambda:time.time()>=planned)
                detach=True  # An explicit mock sensor transition, never PWM-success feedback.
                until(lambda:len(events)==1)
                assert servo.core.physical_released and servo.backend.closed
                assert events[0]['mock'] is True and events[0]['detached_at']==planned
                assert len(received)==1 and abs(received[0]['command_at']-(planned-1.5))<1e-6
            else:
                if mode in ('stop_during','b_ready_lost'):
                    until(lambda:servo.core.state=='COMMAND_SENT')
                if mode=='b_ready_lost':
                    ready=False
                else:
                    a.on_safety_stop('ground fixture operator stop')
                until(lambda:servo.core.state=='FAULT')
                assert a._release_cancelled and servo.backend.closed and not events
                expected=[] if mode=='stop_before' else [1700]
                assert servo.backend.pulses==expected
                a.on_operator_resume()
                pubs['/payload/release_at'].publish(Float64(data=time.time()+1.7))
                spin(.3)
                assert guard.aborted and servo.backend.pulses==expected and not events
            spin(.05)
            assert all(v==0 for v in counts.values()),counts
            assert not hardware_ready,'mock must not advertise real hardware readiness'
            assert all(p.get_subscription_count()>0 for p in (a.pub_cmd,a.pub_mode,a.pub_sp))
            result=dict(mode=mode,passed=True,mock_only=True,flight_outputs=counts,
                        pulses_us=servo.backend.pulses,physical_feedback_events=len(events),
                        release_delay_s=1.5,cancel_latched=a._release_cancelled)
            print('A_GROUND_RESULT='+json.dumps(result,sort_keys=True),flush=True)
            return result
        finally:
            bench.destroy_timer(timer)
            for node in nodes:executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()


def main():
    if (not os.environ.get('PAYLOAD_CATCH_HOST_NETNS')
            or os.readlink('/proc/self/ns/net')==os.environ['PAYLOAD_CATCH_HOST_NETNS']
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise RuntimeError('requires separate network namespace and ROS_LOCALHOST_ONLY=1')
    os.environ['ROS_DOMAIN_ID']='98'
    for mode in ('normal','stop_before','stop_during','b_ready_lost'):scenario(mode)


if __name__=='__main__':main()
