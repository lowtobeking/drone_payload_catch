#!/usr/bin/env python3
"""Two monitor processes communicating over Fast DDS UDP, with fake telemetry.

No flight-control nodes, command publishers, servo nodes or real hardware.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def monitor(role,lock):
    import rclpy
    from payload_catch.fleet_link import FleetLink
    rclpy.init()
    node=FleetLink(cli_args=['--ros-args','-r','__node:=fleet_link_'+role,'-p','role:='+role,
                            '-p','fingerprint:=bench','-p','lock_dir:='+lock])
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


def check():
    from payload_catch.fleet_runtime import dds_profile
    with tempfile.TemporaryDirectory() as temporary:
        profile=Path(temporary)/'dds.xml';profile.write_text(dds_profile('127.0.0.1',['127.0.0.1']))
        os.environ.update(ROS_DOMAIN_ID='98',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                          FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),FASTDDS_DEFAULT_PROFILES_FILE=str(profile))
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from px4_msgs.msg import VehicleStatus
        from payload_catch.px4_iface import qos_in
        rclpy.init();executor=SingleThreadedExecutor();nodes=[];processes=[];logs=[]
        try:
            for role in ('a','b'):
                log=open(Path(temporary)/(role+'.log'),'w+');logs.append(log)
                processes.append(subprocess.Popen([sys.executable,__file__,'--monitor',role,'--lock',temporary+'/'+role],stdout=log,stderr=log))
            # Names/endpoints are dummy graph fixtures, with no actual task behavior.
            fixtures={'safety_supervisor':'/safety/state','release_guard':'/payload/authorized_release',
                      'payload_servo':'/payload/hardware_status','a_node':'/drone_a/release_cmd',
                      'b_node':'/drone_b/ready','relnav_node':'/drone_a/state'}
            for name,topic in fixtures.items():
                n=Node(name);n.create_publisher(String,topic,10);executor.add_node(n);nodes.append(n)
                if name=='payload_servo':n.create_publisher(String,'/payload/release_event',10)
            probe=Node('fleet_test_probe');nodes.append(probe);executor.add_node(probe)
            latest={};last_true={};count={'a':0,'b':0}
            def observed(m,role):
                d=json.loads(m.data);latest[role]=d;count[role]+=1
                if d['ready']:last_true[role]=time.monotonic()
            for role in ('a','b'):
                probe.create_subscription(String,'/fleet/link_'+role,lambda m,r=role:observed(m,r),10)
            telemetry=[probe.create_publisher(VehicleStatus,topic,qos_in()) for topic in
                       ('/fmu/out/vehicle_status_v1','/px4_1/fmu/out/vehicle_status_v1')]
            ids=[1,2]
            def tick():
                for i,pub in enumerate(telemetry):
                    pub.publish(VehicleStatus(timestamp=int(time.time()*1e6),system_id=ids[i]))
            timer=probe.create_timer(.05,tick)
            def until(predicate,timeout=8):
                deadline=time.monotonic()+timeout
                while not predicate() and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.01)
                assert predicate(),dict(latest=latest,counts=count,processes=[p.poll() for p in processes])
            until(lambda:all(latest.get(r,{}).get('ready') for r in ('a','b')))
            ids[1]=1
            until(lambda:all(latest.get(r,{}).get('ready') is False for r in ('a','b')))
            ids[1]=2
            until(lambda:all(latest.get(r,{}).get('ready') for r in ('a','b')))
            duplicate=Node('a_node');nodes.append(duplicate);executor.add_node(duplicate)
            until(lambda:all(latest.get(r,{}).get('ready') is False for r in ('a','b')))
            executor.remove_node(duplicate);duplicate.destroy_node();nodes.remove(duplicate)
            until(lambda:all(latest.get(r,{}).get('ready') for r in ('a','b')))
            processes[1].terminate();processes[1].wait(timeout=3)
            until(lambda:latest.get('a',{}).get('ready') is False,2)
            print('ROS/Fast DDS UDP PASS: two processes, mutual lease, wrong PX4 ID, duplicate node and peer exit block readiness')
            probe.destroy_timer(timer)
        finally:
            for process in processes:
                if process.poll() is None:process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            for node in nodes:executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()
            for log in logs:
                log.flush();log.seek(0);content=log.read();log.close()
                if 'XMLPARSER' in content or 'Error parsing' in content:raise AssertionError(content)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--monitor',choices=['a','b']);p.add_argument('--lock');a=p.parse_args()
    if a.monitor:monitor(a.monitor,a.lock)
    else:check()
