#!/usr/bin/env python3
"""Isolated no-sensor timed actuator integration; mock PWM, no FC control."""
from pathlib import Path
import json
import os
import sys
import tempfile
import time

root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))

def scenario(mode):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String,Float64
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo
    with tempfile.TemporaryDirectory() as directory:
        rclpy.init(args=['--ros-args','-p','enabled:=true','-p','calibrated:=true','-p','backend:=mock',
            '-p','confirmation_mode:=timed','-p','delay_reference:=operator_direct','-p','release_delay_s:=1.5',
            '-p','attempt_record:='+str(Path(directory)/'attempt.json')])
        executor=SingleThreadedExecutor();nodes=[]
        try:
            guard=ReleaseGuard();nodes.append(guard)
            servo=PayloadServo();nodes.append(servo)
            bench=Node('timed_mock_fixture');nodes.append(bench)
            for node in nodes:executor.add_node(node)
            safety=bench.create_publisher(String,'/safety/state',10)
            permit=bench.create_publisher(String,'/payload/release_permit',10)
            schedule=bench.create_publisher(Float64,'/payload/release_at',10)
            completed=[];physical=[];events=[]
            bench.create_subscription(String,'/payload/mock/release_action_complete',lambda m:completed.append(json.loads(m.data)),10)
            bench.create_subscription(String,'/payload/mock/release_event',lambda m:events.append(json.loads(m.data)),10)
            from std_msgs.msg import Bool
            bench.create_subscription(Bool,'/payload/mock/released',lambda m:physical.append(m.data),10)
            def tick():
                stamp=time.time()
                safety.publish(String(data=json.dumps(dict(stamp=stamp,session='timed-fixture',permits={'0':True,'1':True},
                    actions={'0':'run','1':'run'},release_permitted=True))))
                permit.publish(String(data=json.dumps(dict(stamp=stamp,owner=0,allow=True))))
            timer=bench.create_timer(.02,tick)
            def until(predicate,seconds=3):
                end=time.monotonic()+seconds
                while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
                assert predicate(),(mode,servo.core.state,servo.core.reason)
            until(lambda:servo.core.state=='READY')
            # Give permission subscriptions a full fresh delivery before scheduling.
            until(lambda:guard.allow and guard.supervisor is not None)
            planned=time.time()+1.7;schedule.publish(Float64(data=planned))
            until(lambda:servo.core.state=='COMMAND_SENT')
            started=servo.core.command_mono
            assert servo.backend.pulses==[1700] and not physical and not completed
            assert bench.count_publishers('/payload/mock/latch_feedback')==0
            if mode=='normal':
                until(lambda:time.monotonic()-started>=1.4)
                assert not completed and not servo.core.physical_released
                until(lambda:len(completed)==1 and len(events)==1)
                assert servo.core.state=='ACTION_COMPLETE' and servo.backend.closed
                assert len(events)==1 and not physical and not servo.core.physical_released
                assert completed[0]['physically_confirmed'] is False and completed[0]['detached_at'] is None
            else:
                if mode=='operator_stop':servo.core.abort()
                else:bench.destroy_timer(timer);timer=None
                until(lambda:servo.core.state=='FAULT')
                until(lambda:servo.backend.closed)
                assert not completed and not events and not physical
            print('TIMED_SERVO_ROS_RESULT='+json.dumps(dict(mode=mode,passed=True,mock_only=True,
                pulses_us=servo.backend.pulses,physical_events=len(physical),action_events=len(completed),
                final_state=servo.core.state)),flush=True)
        finally:
            for node in reversed(nodes):executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()


def main():
    if (not os.environ.get('PAYLOAD_CATCH_HOST_NETNS')
            or os.readlink('/proc/self/ns/net')==os.environ['PAYLOAD_CATCH_HOST_NETNS']
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise RuntimeError('Separate network namespace and ROS_LOCALHOST_ONLY=1 required')
    for mode in ('normal','operator_stop','supervisor_loss'):scenario(mode)

if __name__=='__main__':main()