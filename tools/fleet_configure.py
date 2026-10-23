#!/usr/bin/env python3
"""Generate per-host DDS environment, without touching OS network settings."""
import argparse
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from payload_catch.fleet_runtime import load_fleet,dds_profile


def check_host(config,role):
    host=config['hosts'][role]
    if socket.gethostname()!=host['hostname']:
        raise ValueError('hostname does not match assigned role')
    addresses=json.loads(subprocess.check_output(['ip','-j','-4','address'],text=True))
    ips={a['local'] for link in addresses for a in link.get('addr_info',[]) if a.get('family')=='inet'}
    if host['ip'] not in ips:raise ValueError('configured IP is not assigned locally')


def generate(path,role,out):
    config,servo,digest=load_fleet(path)
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    peers=[v['ip'] for v in config['hosts'].values()]+config.get('operator_ips',[])
    if role in ('a','b'):local=config['hosts'][role]['ip']
    else:
        local=role
        if local not in config.get('operator_ips',[]):raise ValueError('operator IP must be listed')
    profile=out/'dds.xml'
    profile.write_text(dds_profile(local,peers))
    values=dict(ROS_DOMAIN_ID=str(config['domain_id']),ROS_LOCALHOST_ONLY='0',
                RMW_IMPLEMENTATION='rmw_fastrtps_cpp',FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                FASTDDS_DEFAULT_PROFILES_FILE=str(profile),FLEET_CONFIG=str(Path(path).resolve()),
                FLEET_ROLE=role,FLEET_CONFIG_DIGEST=digest)
    (out/'network.env').write_text(''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in values.items()))
    return values


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('config');p.add_argument('--role',required=True,help='a, b, or listed operator IPv4')
    p.add_argument('--out',required=True);p.add_argument('--check-host',action='store_true')
    a=p.parse_args()
    if a.check_host:
        if a.role not in ('a','b'):p.error('host check requires a/b')
        check_host(load_fleet(a.config)[0],a.role)
    generate(a.config,a.role,a.out)
    print('DDS configuration generated; no system settings changed')
