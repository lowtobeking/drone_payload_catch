#!/usr/bin/env python3
"""Read-only adoption check for existing Agent, MAVLink relay and px4_msgs overlay.

No service start/stop, parameters, arming, setpoints, preflight receipts or PWM writes.
"""
import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from payload_catch.host_runtime import load_profile,environment_text,validate_observed,fingerprint


def run(args):
    p=subprocess.run(args,check=False,capture_output=True,text=True,timeout=8)
    if p.returncode:raise RuntimeError('read-only command failed: '+args[0]+': '+p.stderr.strip()[:200])
    return p.stdout.strip()


def observe(d):
    addresses=json.loads(run(['ip','-j','-4','address']))
    s=dict(hostname=socket.gethostname(),environment_domain=os.environ.get('ROS_DOMAIN_ID','0'),
        environment_rmw=os.environ.get('RMW_IMPLEMENTATION',''),ros_localhost_only=os.environ.get('ROS_LOCALHOST_ONLY','0'),
        overlay_exists=Path(d['px4_msgs_setup']).is_file(),interfaces={link['ifname']:[a['local']
            for a in link.get('addr_info',[]) if a.get('family')=='inet'] for link in addresses})
    s['px4_msgs_commit']=run(['git','-c','safe.directory='+d['px4_msgs_source'],'-C',d['px4_msgs_source'],'rev-parse','HEAD'])
    s['px4_msgs_clean']=not run(['git','-c','safe.directory='+d['px4_msgs_source'],'-C',d['px4_msgs_source'],'status','--porcelain'])
    for kind in ('agent','relay'):
        raw=run(['systemctl','show',d[kind]['unit'],'--property=ActiveState,UnitFileState,MainPID'])
        fields=dict(line.split('=',1) for line in raw.splitlines())
        pid=int(fields['MainPID']);p=Path('/proc')/str(pid)
        row=dict(pid=pid,active=fields['ActiveState'],enabled=fields['UnitFileState'],argv=[],environment={})
        if pid:
            row['argv']=[a.decode() for a in (p/'cmdline').read_bytes().split(b'\0') if a]
            env=dict(a.decode().split('=',1) for a in (p/'environ').read_bytes().split(b'\0') if b'=' in a)
            row['environment']={k:env[k] for k in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','MAV_RELAY_FC','LD_LIBRARY_PATH') if k in env}
        if kind=='relay':row['script_sha256']=fingerprint(d['relay']['script'])
        s[kind]=row
    pids=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:argv=(p/'cmdline').read_bytes().split(b'\0')
        except OSError:continue
        if argv and Path(os.fsdecode(argv[0])).name=='MicroXRCEAgent':pids.append(int(p.name))
    s['agent_pids']=sorted(pids)
    owners={str(d[k]['port']):set() for k in ('agent','relay')}
    for line in run(['ss','-l','-u','-t','-n','-p']).splitlines()[1:]:
        parts=line.split()
        if len(parts)<5:continue
        port=parts[4].rsplit(':',1)[-1]
        if port in owners:owners[port].update(map(int,re.findall(r'pid=(\d+)',line)))
    s['udp_owners']={k:sorted(v) for k,v in owners.items()}
    if d.get('ownership')=='managed':
        prefix=Path(d['managed_prefix'])
        files=[prefix,Path(d['agent']['executable']),Path(d['relay']['script']),
               Path(d['px4_msgs_setup']),Path(d['px4_msgs_source']),prefix/'agent_manifest.json']
        s['managed_tree_protected']=all(p.exists() and p.stat().st_uid==0 and not p.stat().st_mode&0o022 for p in files)
        manifest=json.loads((prefix/'agent_manifest.json').read_text())
        s['agent_bundle_intact']=(fingerprint(d['agent']['executable'])==d['agent']['sha256']==manifest['binary']
            and all(fingerprint(prefix/'lib'/name)==digest for name,digest in manifest['libraries'].items()))
        s['legacy_services']={}
        for name in d['legacy_units']:
            raw=run(['systemctl','show',name,'--property=ActiveState,UnitFileState'])
            state=dict(line.split('=',1) for line in raw.splitlines())
            s['legacy_services'][name]=dict(active=state['ActiveState'],enabled=state['UnitFileState'])
    return s


def ros_observe(d,seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from px4_msgs.msg import VehicleStatus,VehicleLocalPosition,VehicleAttitude,BatteryStatus
    from payload_catch.px4_iface import topic_for
    role_id=0 if d['role']=='a' else 1
    samples={key:[] for key in ('status','position','attitude','battery')}
    rclpy.init();node=Node('jetson_runtime_readonly',enable_rosout=False,start_parameter_services=False)
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    def receive(key,m):
        row=dict(stamp=m.timestamp,received=time.monotonic())
        if key=='status':row.update(system_id=m.system_id,arming_state=m.arming_state)
        samples[key].append(row)
    try:
        for key,cls,suffix in (('status',VehicleStatus,'vehicle_status_v1'),
            ('position',VehicleLocalPosition,'vehicle_local_position'),('attitude',VehicleAttitude,'vehicle_attitude'),
            ('battery',BatteryStatus,'battery_status')):
            node.create_subscription(cls,topic_for(role_id,'out/'+suffix),lambda m,k=key:receive(k,m),qos)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        now=time.monotonic();failures=[];details={}
        for key,rows in samples.items():
            stamps=[r['stamp'] for r in rows]
            age=now-rows[-1]['received'] if rows else None
            ok=len(stamps)>=2 and all(b>a for a,b in zip(stamps,stamps[1:])) and age<=1.
            details[key]=dict(samples=len(rows),latest_age_s=age,source_time_increases=ok)
            if not ok:failures.append('missing/stale/non-increasing real '+key+' telemetry')
        rows=samples['status']
        if not rows or any(r['system_id']!=d['system_id'] or r['arming_state']!=1 for r in rows):
            failures.append('real FC identity/disarmed state not confirmed')
        control={}
        prefix=topic_for(role_id,'in/')
        for name,_ in node.get_topic_names_and_types():
            if name.startswith(prefix):
                writers=node.get_publishers_info_by_topic(name)
                if writers:control[name]=[p.node_namespace+'/'+p.node_name for p in writers]
        if control:failures.append('another ROS controller already publishes to this FC')
        details.update(fcu_system_id=rows[-1]['system_id'] if rows else None,
                       fcu_disarmed=bool(rows) and all(r['arming_state']==1 for r in rows),control_publishers=control)
        return details,failures
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=('environment','check'));p.add_argument('--profile',required=True)
    p.add_argument('--out');p.add_argument('--fleet');p.add_argument('--role',choices=('a','b'))
    p.add_argument('--seconds',type=float,default=4)
    a=p.parse_args()
    if not 2<=a.seconds<=10:p.error('seconds must be 2..10')
    d=load_profile(a.profile)
    if socket.gethostname()!=d['hostname']:raise ValueError('runtime profile belongs to another host')
    if a.role and a.role!=d['role']:raise ValueError('runtime profile belongs to another role')
    if a.command=='environment':
        if not a.out:p.error('--out required')
        Path(a.out).write_text(environment_text(d,a.profile),encoding='utf-8')
        return 0
    fleet=None
    if a.fleet:
        from payload_catch.fleet_runtime import load_fleet
        fleet,_,_=load_fleet(a.fleet)
    observed=observe(d);failures=validate_observed(d,observed,fleet=fleet)
    ros={}
    if not failures:
        ros,additional=ros_observe(d,a.seconds);failures.extend(additional)
    result=dict(passed=not failures,profile=str(Path(a.profile).resolve()),profile_sha256=fingerprint(a.profile),
        reused_existing_services=d.get('ownership')!='managed',managed_runtime=d.get('ownership')=='managed',flight_approval=False,actuator_commands_sent=0,
        observations=observed,ros=ros,failures=failures)
    text=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if a.out:Path(a.out).write_text(text,encoding='utf-8')
    print(text,end='')
    return 1 if failures else 0


if __name__=='__main__':raise SystemExit(main())
