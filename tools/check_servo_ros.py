#!/usr/bin/env python3
"""Localhost ROS guard -> mock latch test; never starts FCU nodes or hardware."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '97'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from payload_catch.release_guard import ReleaseGuard
from payload_catch.payload_servo_node import PayloadServo


def scenario(abort, during=False):
    with tempfile.TemporaryDirectory() as temporary:
        record = str(Path(temporary) / 'attempt.json')
        rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'calibrated:=true',
                         '-p', 'backend:=mock', '-p', 'release_delay_s:=1.5',
                         '-p', 'attempt_record:=' + record])
        executor = SingleThreadedExecutor()
        guard, servo = ReleaseGuard(), PayloadServo()
        sensor = Node('servo_test_sensors')
        nodes = [guard, servo, sensor]
        for node in nodes:
            executor.add_node(node)
        pubs = {topic: sensor.create_publisher(kind, topic, 10) for kind, topic in (
            (String, '/safety/state'), (String, '/payload/release_permit'),
            (String, '/payload/mock/latch_feedback'), (Float64, '/payload/release_at'),
            (Bool, '/safety/pause'), (String, '/payload/authorized_release'))}
        observed, real_ready, envelopes, release_events = [], [], [], []
        sensor.create_subscription(Bool, '/payload/mock/released', lambda m: observed.append(m.data), 10)
        sensor.create_subscription(Bool, '/payload/hardware_ready', lambda m: real_ready.append(m.data), 10)
        sensor.create_subscription(String, '/payload/authorized_release', lambda m: envelopes.append(m.data), 10)
        sensor.create_subscription(String,'/payload/mock/release_event',lambda m:release_events.append(json.loads(m.data)),10)
        detach = False
        def tick():
            stamp = time.time()
            pubs['/safety/state'].publish(String(data=json.dumps(dict(stamp=stamp, session='bench',
                permits={'0': True, '1': True}, actions={'0': 'run', '1': 'run'}, release_permitted=True))))
            pubs['/payload/release_permit'].publish(String(data=json.dumps(dict(stamp=stamp, owner=0, allow=True))))
            feedback = dict(stamp=stamp, session='bench', locked=not detach, present=not detach,
                            detached=detach, healthy=True)
            if detach:
                feedback['detached_at'] = servo.core.planned_at
            pubs['/payload/mock/latch_feedback'].publish(String(data=json.dumps(feedback)))
        timer = sensor.create_timer(.02, tick)
        def until(predicate, timeout=2):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)
            assert predicate(), (servo.core.state, servo.core.reason)
        try:
            until(lambda: servo.core.state == 'READY')
            planned = time.time() + 1.7
            pubs['/payload/release_at'].publish(Float64(data=planned))
            if abort:
                if during:
                    until(lambda: servo.core.state == 'COMMAND_SENT')
                    until(lambda: time.time() >= planned - .7)
                    assert servo.core.hardware_ready(time.time(),time.monotonic())
                pubs['/safety/pause'].publish(Bool(data=True))
                until(lambda: servo.core.state == 'FAULT')
                end = time.monotonic() + .4
                until(lambda: time.monotonic() >= end)
                assert not observed
                assert servo.backend.closed
                if during:
                    assert servo.backend.pulses == [1700] and Path(record).exists()
                else:
                    assert not servo.backend.pulses and not Path(record).exists()
            else:
                until(lambda: servo.core.state == 'COMMAND_SENT')
                assert servo.backend.pulses == [1700] and not observed  # Direct target, no neutral pre-pulse.
                assert servo.core.hardware_ready(time.time(), time.monotonic())
                until(lambda: time.time() >= planned - .1)
                assert not observed and not servo.core.physical_released
                assert servo.core.state == 'COMMAND_SENT' and not servo.backend.closed
                until(lambda: time.time() >= planned)
                detach = True  # Separate mock SENSOR input; no command-derived release in driver.
                until(lambda: observed == [True])
                assert servo.core.state == 'RELEASED', servo.core.reason
                assert servo.backend.closed, 'continuous servo must stop on physical feedback'
                until(lambda:len(release_events)==1)
                assert release_events[0]['detached_at']==planned
                assert release_events[0]['mock'] is True
                until(lambda: bool(envelopes))
                pubs['/payload/authorized_release'].publish(String(data=envelopes[0]))
                end = time.monotonic() + .1
                until(lambda: time.monotonic() >= end)
                assert servo.backend.pulses == [1700] and observed == [True]
                assert abs(json.loads(envelopes[0])['planned_at'] - planned) < 1e-6
                executor.remove_node(servo)
                servo.destroy_node()
                nodes.remove(servo)
                servo = PayloadServo()
                nodes.append(servo)
                executor.add_node(servo)
                assert servo.core.state == 'FAULT'  # Persisted one-shot blocks restart.
                assert not servo.backend.pulses
            assert not real_ready, 'mock driver must not advertise real hardware readiness'
            print('ROS integration PASS:', ('operator stop cancels during opening' if during else 'operator stop cancels before open') if abort else
                  'guard -> PWM mock -> measured detach; replay/restart blocked; real topics isolated')
        finally:
            sensor.destroy_timer(timer)
            for node in nodes:
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    scenario(False)
    scenario(True)
    scenario(True,during=True)
