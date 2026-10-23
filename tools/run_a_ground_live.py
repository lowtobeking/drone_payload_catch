#!/usr/bin/env python3
"""Run installed A task classes on real Jetson telemetry with all FC output isolated.

This bounded ground check never supplies takeoff approval or enables PWM. Its
result is a node-startup verdict, not flight permission. Use run_a_ground_live.sh.
"""
from pathlib import Path
import argparse
import math


def ground_is_fresh(status,land,last_status,last_land,now):
    return (len(status)>=2 and bool(land) and all(land)
            and all(identity==1 and armed==1 for identity,armed,_ in status)
            and all(isinstance(stamp,(int,float)) and math.isfinite(stamp) and stamp>0 for _,_,stamp in status)
            and status[-1][2]>status[0][2]
            and all(isinstance(stamp,(int,float)) and math.isfinite(stamp)
                    and 0<=now-stamp<=2 for stamp in (last_status,last_land)))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',default=str(Path.home()/'payload_catch_deploy'))
    parser.add_argument('--seconds',type=int,default=20,choices=range(5,61),metavar='5..60')
    parser.add_argument('--out',required=True,help='New result.json path; never overwrite an old result')
    options=parser.parse_args(argv)
    if not __debug__:raise RuntimeError('Python optimization is prohibited for this ground checker')
    output_path=Path(options.out).resolve()
    if output_path.exists():parser.error('Result already exists; select a new report directory')
    output_path.parent.mkdir(parents=True,exist_ok=True)

    import fcntl
    import inspect
    import json
    import os
    import signal
    from collections import deque
    import socket
    import subprocess
    import sys
    import time
    import yaml

    base=Path(options.base).resolve()
    if os.uname().machine!='aarch64':raise RuntimeError('Run on A Jetson, not the development computer')
    if (base/'ROLE').read_text().strip()!='a':raise RuntimeError('A role required')
    root=(base/'current/src').resolve()
    subprocess.run(['/usr/bin/python3',str(root/'tools/fleet_deploy.py'),'verify',str(root)],check=True,stdout=subprocess.DEVNULL,timeout=10)
    sys.path.insert(0,str(root))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from px4_msgs.msg import VehicleCommand,OffboardControlMode,TrajectorySetpoint,VehicleStatus,VehicleLandDetected
    from std_msgs.msg import String
    from payload_catch.a_node import ANode
    from payload_catch.safety_supervisor import SafetySupervisor
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo

    cfg=yaml.safe_load((root/'config/payload_servo.yaml').read_text())['payload_servo']['ros__parameters']
    assert cfg['enabled'] is False and cfg['calibrated'] is False and cfg['backend']=='jetson'
    no_approval='/dev/null'  # Cannot contain a valid JSON approval; never written by this tool.
    classes=(ANode,SafetySupervisor,ReleaseGuard,PayloadServo)
    assert all(Path(inspect.getfile(cls)).resolve().is_relative_to(root) for cls in classes)
    args=['--ros-args','--params-file',str(root/'config/payload_servo.yaml')]
    for param in ('auto_arm:=false','preflight_report:='+str(no_approval),'require_fleet_link:=true','coord_mode:=handshake',
                  'release_lead:=1.7','release_delay_s:=1.5','publish_state:=false','auto_land:=false','require_rc:=true',
                  'preflight_require_rc:=true','preflight_require_gps:=true','enabled:=false','calibrated:=false','release_id:='+root.parent.name):
        args.extend(['-p',param])
    topics={}
    for prefix in ('/fmu','/px4_1/fmu'):
        for name in ('vehicle_command','offboard_control_mode','trajectory_setpoint'):
            original=prefix+'/in/'+name
            target='/payload_ground_check'+original
            args.extend(['-r',original+':='+target])
            topics[original]=target
    stop=False
    def stopping(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,stopping)
    lock=(base/'mission.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init(args=args);executor=SingleThreadedExecutor();nodes=[]
    probe=Node('a_ground_start_monitor',enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
    nodes.append(probe);executor.add_node(probe)
    q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    status=deque(maxlen=256);land=deque(maxlen=256);hardware=deque(maxlen=256);safety=deque(maxlen=256)
    last_status=[None];last_land=[None];land_stamps=[0]
    status_count=[0];hardware_count=[0];safety_count=[0];counts={topic:0 for pair in topics.items() for topic in pair};counts['/payload/authorized_release']=0

    def count(topic):
        def callback(_):counts[topic]+=1
        return callback
    for original,target in topics.items():
        kind=VehicleCommand if original.endswith('vehicle_command') else OffboardControlMode if original.endswith('offboard_control_mode') else TrajectorySetpoint
        for topic in (original,target):probe.create_subscription(kind,topic,count(topic),q)
    probe.create_subscription(String,'/payload/authorized_release',count('/payload/authorized_release'),10)
    def observed_hardware(m):
        hardware.append(json.loads(m.data));hardware_count[0]+=1
    probe.create_subscription(String,'/payload/hardware_status',observed_hardware,10)
    def observed_safety(m):
        safety.append(json.loads(m.data));safety_count[0]+=1
    probe.create_subscription(String,'/safety/state',observed_safety,10)
    def observed_status(m):
        if m.timestamp<=0 or (status and m.timestamp<=status[-1][2]):return
        status.append((m.system_id,m.arming_state,m.timestamp));last_status[0]=time.monotonic();status_count[0]+=1
    probe.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',observed_status,q)
    def observed_land(m):
        if m.timestamp<=land_stamps[0]:return
        land.append(m.landed);land_stamps[0]=m.timestamp;last_land[0]=time.monotonic()
    probe.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',observed_land,q)

    def live_guard():
        now=time.monotonic()
        if not ground_is_fresh(status,land,last_status[0],last_land[0],now):raise RuntimeError('Ground/disarmed telemetry missing, stale, wrong identity, or aircraft armed')
        if any(counts.values()):raise RuntimeError('Unexpected control/release output: '+str(counts))

    def spin(seconds,guarded=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not stop:
            executor.spin_once(timeout_sec=.02)
            if guarded:live_guard()

    def pwm():
        raw=subprocess.run(['sudo','-n','cat','/sys/kernel/debug/pwm'],check=True,capture_output=True,text=True,timeout=5).stdout
        blocks=[block for block in raw.split('\n\n') if block.startswith('platform/32c0000.pwm,')]
        assert len(blocks)==1 and 'enabled' not in blocks[0]
        return blocks[0]

    try:
        spin(3)
        assert ground_is_fresh(status,land,last_status[0],last_land[0],time.monotonic())
        assert not stop,'interrupted before startup'
        assert status[-1][2]>status[0][2]
        assert not any(probe.count_publishers(topic) for topic in topics),'existing FC control publishers'
        assert not any(name in ('a_node','b_node','safety_supervisor','release_guard','payload_servo') for name,_ in probe.get_node_names_and_namespaces())
        pwm_before=pwm()
        for cls in classes[1:]:
            node=cls();nodes.append(node);executor.add_node(node)
        supervisor,guard,servo=nodes[1:]
        a=ANode();nodes.append(a);executor.add_node(a)
        assert a.pub_cmd.topic_name==topics['/fmu/in/vehicle_command']
        assert a.pub_mode.topic_name==topics['/fmu/in/offboard_control_mode']
        assert a.pub_sp.topic_name==topics['/fmu/in/trajectory_setpoint']
        for ident,prefix in ((0,'/fmu'),(1,'/px4_1/fmu')):
            assert supervisor.command_pub[ident].topic_name==topics[prefix+'/in/vehicle_command']
        assert not a.auto_arm and a.require_fleet_link and not servo.core.config.enabled
        started=time.monotonic()
        print('JETSON_GROUND_TASK_STARTED='+json.dumps(dict(host=socket.gethostname(),pid=os.getpid(),release=root.parent.name,nodes=[node.get_name() for node in nodes[1:]],real_telemetry=True,auto_arm=False,physical_pwm=False)),flush=True)
        for step in range(options.seconds):
            spin(1,guarded=True)
            assert not stop,'interrupted'
            assert not any(probe.count_publishers(topic) for topic in topics)
            assert not any(counts.values()),counts
            assert not a.mission_allowed() and not a._takeoff_ready() and a._tick_count==0
            assert all(i==1 and armed==1 for i,armed,_ in status)
            if (step+1)%5==0:print('JETSON_GROUND_TASK_PROGRESS='+json.dumps(dict(elapsed_s=round(time.monotonic()-started,2),status_samples=status_count[0],hardware_state=servo.core.state,mission_allowed=a.mission_allowed())),flush=True)
        assert hardware and all(row['ready'] is False and row['state']=='DISABLED' for row in hardware)
        assert safety and not any(row.get('release_permitted') is True for row in safety)
        assert servo.core.state=='DISABLED' and not servo.backend.pulses
        assert pwm()==pwm_before and all(land)
        result=dict(passed=True,scope='real A task nodes on Jetson, ground startup only',real_flight_completed=False,
            hostname=socket.gethostname(),architecture=os.uname().machine,pid=os.getpid(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            release=root.parent.name,duration_s=time.monotonic()-started,real_telemetry=True,source_files={cls.__name__:inspect.getfile(cls) for cls in classes},
            nodes=[node.get_name() for node in nodes[1:]],auto_arm=False,flight_approval=False,required_fleet_link=True,flight_gate_allowed=False,
            control_output_remaps=topics,observed_counts=counts,status_samples=status_count[0],physical_pwm_output=False,
            pwm_controller=pwm_before,hardware_status_samples=hardware_count[0],servo_state=servo.core.state,
            safety_status_samples=safety_count[0],safety_permits=safety[-1].get('permits'),safety_reasons=safety[-1].get('reasons'),
            a_sensor_frames={ident:{key:len(queue) for key,queue in health.samples.items()} for ident,health in a._takeoff_gate.health.items()})
        from payload_catch.preflight_health import evaluate
        result['health']={str(ident):evaluate(health.snapshot(time.monotonic()),require_gps=True,require_rc=True) for ident,health in a._takeoff_gate.health.items()}
        result['note']='Ground startup check only; passed does not mean flight ready'
        output_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('JETSON_GROUND_TASK_RESULT='+json.dumps(result,ensure_ascii=False),flush=True)
    except BaseException as exc:
        output_path.write_text(json.dumps(dict(passed=False,flight_approval=False,error=type(exc).__name__+': '+str(exc)),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        raise
    finally:
        for node in reversed(nodes):
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown();lock.close()
        print('JETSON_GROUND_TASK_CLEANED',flush=True)

if __name__=="__main__":main()
