#!/usr/bin/env python3
"""Actual A/guard/servo/supervisor in a fake-PX4 kinematic closed loop.

Requires a separate netns, loopback-only network, ordinary user and MOCK PWM.
No real PX4, mechanical feedback, flight dynamics or calibrated hardware implied.
"""
from pathlib import Path
import json
import math
import os
import socket
import signal
import subprocess
import sys
import tempfile
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))



def _stop_components(workers):
    for process in workers:
        if process.poll() is None: process.terminate()
    for process in workers:
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill();process.wait(timeout=5)


def _component():
    assert os.geteuid()!=0 and os.environ['ROS_LOCALHOST_ONLY']=='1'
    assert os.readlink('/proc/self/ns/net')!=os.environ['PAYLOAD_CATCH_HOST_NETNS']
    assert {name for _,name in socket.if_nameindex()}=={'lo'}
    import rclpy
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo
    from payload_catch.safety_supervisor import SafetySupervisor
    kind=sys.argv[2];summary=Path(sys.argv[3]);stop=False
    def stopping(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,stopping)
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=json.loads(os.environ['PAYLOAD_CATCH_MOCK_ROS_ARGS']),
               signal_handler_options=SignalHandlerOptions.NO)
    node={'guard':ReleaseGuard,'servo':PayloadServo,'supervisor':SafetySupervisor}[kind]()
    if kind=='servo':assert node.core.mock and type(node.backend).__name__=='MockPWM'
    summary.with_suffix('.ready').write_text(kind)
    try:
        deadline=time.monotonic()+65
        while not stop and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        node.destroy_node()
        if kind=='servo':
            summary.write_text(json.dumps(dict(state=node.core.state,reason=node.core.reason,
                pulses=node.backend.pulses,closed=node.backend.closed,physical_released=node.core.physical_released,
                fault_reported=node.fault_reported,mock=node.core.mock)))
        if rclpy.ok():rclpy.shutdown()


def _start_components(args,temporary):
    workers=[]
    env=dict(os.environ,PAYLOAD_CATCH_MOCK_ROS_ARGS=json.dumps(args))
    try:
        for kind in ('guard','servo','supervisor'):
            workers.append(subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--component',
                kind,str(Path(temporary)/(kind+'.json'))],env=env))
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in workers):raise RuntimeError('mock component startup failed')
            if all((Path(temporary)/(kind+'.ready')).exists() for kind in ('guard','servo','supervisor')):
                return workers
            time.sleep(.02)
        raise TimeoutError('mock component startup timeout')
    except BaseException:
        _stop_components(workers)
        raise


def main():
    if (os.geteuid()==0 or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not os.environ.get('PAYLOAD_CATCH_HOST_NETNS')
            or os.readlink('/proc/self/ns/net')==os.environ['PAYLOAD_CATCH_HOST_NETNS']
            or {name for _,name in socket.if_nameindex()}!={'lo'}):
        raise RuntimeError('requires unprivileged, separate loopback-only netns')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from std_msgs.msg import Bool,String,Float64MultiArray
    from px4_msgs.msg import (VehicleStatus,VehicleLocalPosition,VehicleAttitude,
        BatteryStatus,FailsafeFlags,EstimatorStatusFlags,SensorGps,ManualControlSetpoint,
        VehicleLandDetected,VehicleCommand,VehicleCommandAck,TrajectorySetpoint,OffboardControlMode)
    from payload_catch.a_node import ANode
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo
    from payload_catch.safety_supervisor import SafetySupervisor
    from payload_catch.preflight_health import evaluate,BATTERY_MIN
    from payload_catch.fcu_params import check_params,PRESETS
    from payload_catch.px4_iface import topic_for

    report=ROOT/'report/a_full_mission_20261016';report.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='a-mission-mock-') as temporary:
        receipt=Path(temporary)/'MOCK_ONLY_preflight.json'
        ros_args=['--ros-args','--log-level','warn',
            '-p','auto_arm:=true','-p','coord_mode:=handshake','-p','require_fleet_link:=true',
            '-p','preflight_report:='+str(receipt),'-p','preflight_require_gps:=true',
            '-p','preflight_require_rc:=true','-p','require_rc:=true',
            '-p','hover_world:=[0.0,0.0,-2.5]','-p','land_xy:=[-2.0,0.0]',
            '-p','clear_offset:=[2.0,0.0,0.0]','-p','auto_land:=true',
            '-p','land_after_catch_s:=1.0','-p','release_lead:=1.7','-p','release_delay_s:=1.5',
            '-p','backend:=mock','-p','enabled:=true','-p','calibrated:=true',
            '-p','attempt_record:='+str(Path(temporary)/'mock_attempt.json')]
        rclpy.init(args=ros_args)
        a=ANode()
        sim=Node('a_full_mission_fake_px4',enable_rosout=False,start_parameter_services=False)
        workers=_start_components(ros_args,temporary);servo_status={}
        executor=SingleThreadedExecutor();nodes=[a,sim]
        for node in nodes:executor.add_node(node)
        q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        monitor=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        telemetry={}
        kinds=dict(status=(VehicleStatus,'vehicle_status_v1'),lpos=(VehicleLocalPosition,'vehicle_local_position'),
            att=(VehicleAttitude,'vehicle_attitude'),batt=(BatteryStatus,'battery_status'),
            failsafe=(FailsafeFlags,'failsafe_flags'),est=(EstimatorStatusFlags,'estimator_status_flags'),
            gps=(SensorGps,'vehicle_gps_position'),rc=(ManualControlSetpoint,'manual_control_setpoint'),
            land=(VehicleLandDetected,'vehicle_land_detected'),ack=(VehicleCommandAck,'vehicle_command_ack'))
        for i in (0,1):
            for key,(cls,suffix) in kinds.items():telemetry[i,key]=sim.create_publisher(cls,topic_for(i,'out/'+suffix),q)
        pubs={name:sim.create_publisher(kind,name,10) for kind,name in (
            (String,'/safety/task_b'),(String,'/fleet/link_a'),(Float64MultiArray,'/drone_b/ready'),
            (String,'/payload/mock/latch_feedback'),(Bool,'/payload/released'),
            (Bool,'/payload/caught'))}
        p=[np.array([0.,0.,0.]),np.array([0.,0.,0.])]
        v=[np.zeros(3),np.zeros(3)];armed=[False,False];nav=[4,4];landing=[False,False]
        sp=np.zeros(3);sp_at=0.;events=[];commands=[];modes=[];setpoints=[];release_events=[]
        supervisor_states=[];real_ready=[];detached=False;detached_at=None;caught=False
        counts=dict(missing_blocked=0,low_battery_blocked=0)
        begin=time.monotonic();last_step=begin;receipt_written=False;clear_done=False
        phases=set();latest_health={};last_telemetry=-float("inf");last_feedback=-float("inf");authorizations=[]
        def event(name,**data):
            if name in phases:return
            phases.add(name);row=dict(stage=name,elapsed_s=round(time.monotonic()-begin,3),**data)
            events.append(row);print('A_MISSION_STAGE '+json.dumps(row,ensure_ascii=False),flush=True)
        def observe_command(m,i):
            row=dict(elapsed_s=time.monotonic()-begin,drone=i,command=m.command,param1=m.param1,param2=m.param2)
            commands.append(row)
            if m.command==400 and m.param1==1.:
                assert receipt_written and a._takeoff_gate.good_since is not None
                assert time.monotonic()-a._takeoff_gate.good_since>=3.,'stable preflight window bypassed'
                armed[i]=True;event('simulated_arm_accepted',drone=i)
            elif m.command==176:
                nav[i]=14 if m.param2==6. else 4
            elif m.command==21:
                landing[i]=True;nav[i]=18;event('simulated_land_command_'+str(i))
            else:
                raise AssertionError('unexpected command: '+str(row))
            ack=VehicleCommandAck(timestamp=int(time.time()*1e6),command=m.command,result=0,
                                   target_system=m.source_system,target_component=m.source_component,from_external=True)
            telemetry[i,'ack'].publish(ack)
        def observe_sp(m):
            nonlocal sp,sp_at
            setpoints.append(dict(elapsed_s=time.monotonic()-begin,velocity=list(m.velocity)))
            sp=np.array(m.velocity,float);sp_at=time.monotonic()
        sim.create_subscription(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',observe_sp,monitor)
        sim.create_subscription(OffboardControlMode,'/fmu/in/offboard_control_mode',lambda m:modes.append(m.timestamp),monitor)
        for i in (0,1):sim.create_subscription(VehicleCommand,topic_for(i,'in/vehicle_command'),
            lambda m,i=i:observe_command(m,i),monitor)
        def authorization_observed(m):
            data=json.loads(m.data);wall=time.time()
            authorizations.append(dict(received_at=wall,command_age_s=wall-data['command_at'],
                authorization_age_s=wall-data['stamp'],
                feedback_age_s=wall-servo_status.get('feedback_stamp',wall),
                supervisor_age_s=wall-(supervisor_states[-1]['stamp'] if supervisor_states else wall)))
        sim.create_subscription(String,'/payload/authorized_release',authorization_observed,10)
        def release(m):
            release_events.append(json.loads(m.data))
            pubs['/payload/released'].publish(Bool(data=True))  # Explicit mock bridge within isolated netns only.
        sim.create_subscription(String,'/payload/mock/release_event',release,10)
        def latch_status(m):
            servo_status.clear();servo_status.update(json.loads(m.data))
        sim.create_subscription(String,'/payload/mock/hardware_status',latch_status,10)
        sim.create_subscription(String,'/safety/state',lambda m:supervisor_states.append(json.loads(m.data)),10)
        sim.create_subscription(Bool,'/payload/hardware_ready',lambda m:real_ready.append(m.data),10)
        def make_health(i,key,stamp,remaining):
            cls=kinds[key][0];m=cls();m.timestamp=stamp
            if key=='status':
                m.system_id=i+1;m.arming_state=2 if armed[i] else 1;m.nav_state=nav[i]
                m.pre_flight_checks_pass=True
            elif key=='lpos':
                m.x,m.y,m.z=map(float,p[i]);m.vx,m.vy,m.vz=map(float,v[i])
                m.xy_valid=m.z_valid=m.v_xy_valid=m.v_z_valid=True
                m.heading_good_for_control=True;m.dead_reckoning=False;m.eph=.05;m.epv=.05
                m.vxy_max=m.vz_max=m.hagl_min=float('inf')
            elif key=='att':m.q=[1.,0.,0.,0.]
            elif key=='batt':m.connected=True;m.remaining=remaining;m.voltage_v=16.;m.warning=0;m.cell_count=4
            elif key=='est':m.cs_tilt_align=True;m.cs_yaw_align=True
            elif key=='gps':m.eph=.5;m.epv=.7;m.satellites_used=25;m.fix_type=3
            elif key=='rc':m.valid=True;m.data_source=1
            elif key=='land':m.landed=bool(not armed[i]);m.maybe_landed=m.landed;m.ground_contact=m.landed
            return m
        # Parameter fixture is checked by the actual complete safety-parameter evaluator.
        params={name:value for group in ('failsafe','thrust','limits_indoor','battery','geofence_outdoor','ekf2_rtk')
                for name,value,_ in PRESETS[group]}
        params.update(MPC_THR_HOVER=.3,GF_ACTION=4,SENS_BOARD_ROT=0,COM_RC_IN_MODE=0,
                      RC_MAP_KILL_SW=7,RC_MAP_FLTMODE=5,**{'COM_FLTMODE'+str(i):0 for i in range(1,7)})
        param_results=check_params(params,'outdoor',require_complete=True)
        assert not any(x.level=='fail' for x in param_results),param_results
        def tick():
            nonlocal sp,last_step,receipt_written,detached,detached_at,caught,clear_done,last_telemetry,last_feedback
            now=time.monotonic();elapsed=now-begin;dt=min(now-last_step,.05);last_step=now
            wall=time.time();stamp=int(wall*1e6)
            if armed[0] and nav[0]==14 and now-sp_at<.15:
                v[0]=sp.copy();p[0]+=v[0]*dt
            elif not landing[0]:v[0]=np.zeros(3)
            if armed[0] and not armed[1] and not landing[1]:
                armed[1]=True;nav[1]=14;p[1]=np.array([0.,0.,-1.5])
            for i in (0,1):
                if landing[i] and armed[i]:
                    v[i]=np.array([0.,0.,.6]);p[i]+=v[i]*dt
                    if p[i][2]>=0:
                        p[i][2]=0.;v[i][:]=0.;armed[i]=False
                        event('simulated_touchdown_disarmed_'+str(i))
            remaining=.39 if .8<=elapsed<1.6 else .85
            # Synthetic telemetry at 20Hz is sufficient for the unchanged health gates.
            # Flooding every topic at 100Hz overloads the single-process fake-FC loop
            # on the Jetson and obscures the 20/30ms production release deadlines.
            if now-last_telemetry>=.05:
                last_telemetry=now
                for i in (0,1):
                    if i==1 and elapsed<.8:continue
                    for key in kinds:
                        if key=='ack':continue
                        telemetry[i,key].publish(make_health(i,key,stamp,remaining))
            if elapsed<.8:
                assert not commands and not setpoints and not modes
                counts['missing_blocked']+=1
            elif elapsed<1.6:
                assert not commands and not setpoints and not modes
                counts['low_battery_blocked']+=1
            if elapsed>=1.7 and not receipt_written:
                latest_health.clear()
                for i,acc in a._takeoff_gate.health.items():
                    latest_health[i]=evaluate(acc.snapshot(now),require_gps=True,require_rc=True)
                if all(not any(r['status']=='fail' for r in rows) for rows in latest_health.values()):
                    receipt.write_text(json.dumps(dict(schema=1,passed=True,checked_at=wall,
                        drone_ids=[0,1],battery_min=BATTERY_MIN,gps=True,rc=True,params_checked=[0,1],
                        mock=True,source='isolated synthetic telemetry and synthetic checked parameters')))
                    receipt_written=True;event('mock_selfcheck_passed')
            if now-last_feedback>=.02:
                pubs['/fleet/link_a'].publish(String(data=json.dumps(dict(stamp=wall,ready=True,mock=True))))
                pubs['/safety/task_b'].publish(String(data=json.dumps(dict(stamp=wall,
                    ever_offboard=armed[1] and nav[1]==14,safety='OK',sources_ready=True,released=detached))))
            settled=bool(armed[0] and np.linalg.norm(p[0]-a.hover)<.12 and np.linalg.norm(v[0])<.15)
            if settled and not detached:
                event('arrived_at_release_point',position=p[0].tolist())
                pubs['/drone_b/ready'].publish(Float64MultiArray(data=[1.,.01,.01,wall,.01,.01,1.]))
            if servo_status.get('command_sent_at') is not None:
                event('mock_pwm_1700_applied',planned_detach_at=servo_status.get('planned_detach_at'))
                # Ideal mock plant detaches 1.5s after the accepted output command.
                # Its edge timestamp is independent of when this executor observes it,
                # matching the production sensor-edge protocol (not receive-time).
                mock_edge_at=servo_status.get('command_sent_at')+1.5
                if not detached and wall>=mock_edge_at:
                    detached=True;detached_at=mock_edge_at
                    event('mock_sensor_detachment',opening_delay_s=detached_at-servo_status.get('command_sent_at'),
                          feedback_publish_lag_s=wall-detached_at)
            feedback=dict(stamp=wall,session=(supervisor_states[-1]['session'] if supervisor_states else 'awaiting_session'),locked=not detached,present=not detached,
                          detached=detached,healthy=True)
            if detached:feedback['detached_at']=detached_at
            if now-last_feedback>=.02 or detached and servo_status.get('state','STARTING')=='COMMAND_SENT':
                pubs['/payload/mock/latch_feedback'].publish(String(data=json.dumps(feedback)))
                last_feedback=now
            if detached and p[0][0]>1.85:
                clear_done=True;event('cleared_release_area',position=p[0].tolist())
            if clear_done and not caught:
                pubs['/payload/caught'].publish(Bool(data=True));caught=True
                event('mock_B_caught_report')
        timer=sim.create_timer(.01,tick)
        success=False
        try:
            event('begin_mock_preflight')
            deadline=begin+48
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.005)
                assert servo_status.get('state','STARTING')!='FAULT',dict(reason=servo_status.get('reason',''),authorizations=authorizations)
                if 'simulated_touchdown_disarmed_0' in phases:
                    end=time.monotonic()+.5
                    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
                    success=True;break
            sim.destroy_timer(timer);timer=None
            _stop_components(workers)
            assert all(p.returncode==0 for p in workers),'mock component shutdown failed'
            servo_summary=json.loads((Path(temporary)/'servo.json').read_text())
            assert servo_summary['mock'] and servo_summary['state']=='RELEASED',servo_summary
            assert success,dict(phases=sorted(phases),position=p[0].tolist(),a_landing=a._landing,
                servo=(servo_status.get('state','STARTING'),servo_status.get('reason','')),actions=(supervisor_states[-1]['actions'] if supervisor_states else {}),
                safety=a._safety_reason,approval=a._approval_error)
            assert all(counts.values()) and receipt_written
            assert len(release_events)==1 and servo_summary['pulses']==[1700]
            assert servo_status.get('physically_released',False) and servo_summary['closed']
            assert servo_status.get('state','STARTING')=='RELEASED' and not servo_summary['fault_reported'],'normal landing must preserve release success'
            assert abs(detached_at-servo_status.get('planned_detach_at'))<=.02
            assert not real_ready and not armed[0] and p[0][2]==0 and clear_done
            assert not any(x.get('actions',{}).get('0')=='kill' for x in supervisor_states)
            order=[e['stage'] for e in events]
            assert order.index('simulated_touchdown_disarmed_1')<order.index('simulated_land_command_0')
            assert order.index('mock_sensor_detachment')<order.index('cleared_release_area')<order.index('simulated_land_command_0')
            result=dict(passed=True,simulation_only=True,network_isolated=True,physical_pwm_output=False,
                real_vehicle_commands=0,model='kinematic fake PX4, ideal 1.5s mock plant; synthetic B/GPS/RC; actual mission nodes in separate processes',
                events=events,negative_gate_samples=counts,commands=commands,health_checks=latest_health,
                parameter_checks=[i.__dict__ for i in param_results],
                authorization_delivery=authorizations,mock_pulses_us=servo_summary['pulses'],mock_opening_delay_s=detached_at-servo_status.get('command_sent_at'),
                final_servo_state=servo_status.get('state','STARTING'),servo_fault_reported=servo_summary['fault_reported'],
                final_position=p[0].tolist(),final_armed=False,final_nav_state=nav[0])
            (report/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print('A_FULL_MISSION_RESULT='+json.dumps({k:v for k,v in result.items()
                if k not in ('commands','health_checks','parameter_checks','events')},ensure_ascii=False),flush=True)
        finally:
            if timer is not None:sim.destroy_timer(timer)
            _stop_components(workers)
            for node in nodes:executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--component':_component()
    else:main()
