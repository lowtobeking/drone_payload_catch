"""Verified reuse of existing Jetson communication services; no service mutations."""
import hashlib
import ipaddress
from pathlib import Path
import re
import shlex


def load_profile(path):
    import yaml
    d=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(d,dict) or d.get('schema')!=1 or d.get('role') not in ('a','b'):
        raise ValueError('host profile schema/role required')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+',d.get('hostname','')):
        raise ValueError('actual Jetson hostname required')
    if type(d.get('domain_id')) is not int or not 0<=d['domain_id']<=100:
        raise ValueError('host ROS domain must be 0..100')
    if d.get('system_id') != (1 if d['role']=='a' else 2):
        raise ValueError('role and PX4 system ID mismatch')
    for key in ('px4_msgs_setup','px4_msgs_source'):
        if not Path(d[key]).is_absolute():raise ValueError('absolute message overlay paths required')
    if not re.fullmatch(r'[0-9a-f]{40}',d['px4_msgs_commit']):raise ValueError('message commit required')
    for key in ('interface','local_ip','fc_ip','mesh_ip'):
        if not isinstance(d['network'][key],str):raise ValueError('network fields required')
    for key in ('local_ip','fc_ip','mesh_ip'):
        ip=ipaddress.IPv4Address(d['network'][key])
        if ip.is_loopback or ip.is_unspecified or ip.is_multicast:raise ValueError('actual network addresses required')
    for kind in ('agent','relay'):
        service=d[kind]
        if not re.fullmatch(r'[A-Za-z0-9_.@-]+\.service',service['unit']):raise ValueError('systemd service required')
        if type(service['port']) is not int or not 1<=service['port']<=65535:raise ValueError('UDP port required')
    if not Path(d['relay']['script']).is_absolute():raise ValueError('absolute existing relay path required')
    if not re.fullmatch(r'[0-9a-f]{64}',d['relay']['sha256']):raise ValueError('relay hash required')
    ipaddress.IPv4Address(d['relay']['ground_station_ip'])
    if d.get('ownership')=='managed':
        prefix=Path(d['managed_prefix'])
        if not prefix.is_absolute() or prefix.parts[:3]!=('/','opt','payload_catch'):
            raise ValueError('managed runtime requires protected /opt/payload_catch path')
        for value in (d['px4_msgs_setup'],d['px4_msgs_source'],d['agent']['executable'],d['relay']['script']):
            path=Path(value)
            if '..' in path.parts or not path.is_relative_to(prefix):
                raise ValueError('managed runtime cannot reference an old project')
        if not all(d[k]['unit'].startswith('payload-catch-') for k in ('agent','relay')):
            raise ValueError('project-owned services required')
        if d.get('legacy_units')!=['microxrce-agent.service','mav-relay.service']:
            raise ValueError('explicit old service exclusion required')
    return d


def environment(profile,path):
    return dict(ROS_DOMAIN_ID=str(profile['domain_id']),ROS_LOCALHOST_ONLY='0',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',PX4_MSGS_SETUP=profile['px4_msgs_setup'],
        FLEET_HOST_PROFILE=str(Path(path).resolve()))


def environment_text(profile,path):
    # Remove inherited simulation restrictions. Fleet startup generates its own
    # verified per-host DDS profile after loading this host environment.
    return ('unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE\n'+
        ''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in environment(profile,path).items()))


def validate_observed(d,s,*,fleet=None):
    failures=[]
    def require(ok,reason):
        if not ok:failures.append(reason)
    require(s.get('hostname')==d['hostname'],'host identity differs from runtime profile')
    require(s.get('overlay_exists') is True,'px4_msgs overlay missing')
    require(s.get('px4_msgs_commit')==d['px4_msgs_commit'],'px4_msgs commit differs from validated version')
    require(s.get('px4_msgs_clean') is True,'px4_msgs source has local modifications')
    require(s.get('environment_domain')==str(d['domain_id']),'runtime ROS_DOMAIN_ID differs from verified agent domain')
    require(s.get('environment_rmw')=='rmw_fastrtps_cpp','Fast DDS runtime required')
    require(s.get('ros_localhost_only')=='0','real Jetson runtime cannot use localhost-only DDS')
    addresses=s.get('interfaces',{})
    require(d['network']['local_ip'] in addresses.get(d['network']['interface'],[]),'FC Ethernet address/interface mismatch')
    require(any(d['network']['mesh_ip'] in ips for ips in addresses.values()),'configured mesh interface address missing')
    for kind in ('agent','relay'):
        observed=s.get(kind,{})
        expected=d[kind]
        pid=observed.get('pid',0)
        require(observed.get('active')=='active' and pid>0,kind+' existing service is not running')
        require(observed.get('enabled')=='enabled',kind+' existing service is not enabled at boot')
        require(s.get('udp_owners',{}).get(str(expected['port']))==[pid] and pid>0,
                kind+' UDP port has missing/foreign/multiple owner')
    a=s.get('agent',{});argv=a.get('argv',[])
    require(len(argv)>=4 and argv[0]==d['agent'].get('executable','/usr/local/bin/MicroXRCEAgent') and
            argv[1:]==['udp4','-p',str(d['agent']['port'])], 'Agent executable/transport/port mismatch')
    require(a.get('environment',{}).get('ROS_DOMAIN_ID')==str(d['domain_id']), 'Agent ROS domain differs')
    require(a.get('environment',{}).get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp','Agent DDS implementation differs')
    require(s.get('agent_pids')==[a.get('pid')],'duplicate or unrecognized MicroXRCEAgent process')
    relay=s.get('relay',{});argv=relay.get('argv',[])
    expected=[d['relay']['script'],d['relay']['ground_station_ip']]
    if d.get('ownership')=='managed':
        expected=[d['relay']['script'],'--fc-ip',d['network']['fc_ip'],
                  '--gcs-ip',d['relay']['ground_station_ip'],'--port',str(d['relay']['port'])]
    require(len(argv)>1 and argv[1:]==expected,'relay script/FC/ground station destination mismatch')
    require(relay.get('script_sha256')==d['relay']['sha256'],'existing relay source changed')
    if d.get('ownership')=='managed':
        require(s.get('managed_tree_protected') is True,'managed runtime files are not root-owned/protected')
        require(s.get('agent_bundle_intact') is True,'managed Agent executable/libraries changed')
        require(a.get('environment',{}).get('LD_LIBRARY_PATH')==d['managed_prefix']+'/lib',
                'Agent must use project-owned libraries')
        legacy=s.get('legacy_services',{})
        for name in d['legacy_units']:
            require(legacy.get(name)==dict(active='inactive',enabled='masked'),'old service not excluded: '+name)
    else:
        require(relay.get('environment',{}).get('MAV_RELAY_FC')==d['network']['fc_ip'],'relay points to another FC')
    if fleet is not None:
        require(fleet['domain_id']==d['domain_id'],'fleet domain differs from reused Agent; do not change FC parameters implicitly')
        host=fleet['hosts'][d['role']]
        require(host['hostname']==d['hostname'] and host['system_id']==d['system_id'] and
                host['ip']==d['network']['mesh_ip'],'fleet role/host/mesh address differs from runtime profile')
    return failures


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
