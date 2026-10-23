#!/usr/bin/env python3
"""Exact, reversible ownership transfer. Never sends flight or PWM commands."""
from pathlib import Path
import argparse
import contextlib
import fcntl
import hashlib
import json
import os
import pwd
import re
import shutil
import stat
import subprocess
import sys
import time

PREFIX=Path('/opt/payload_catch/managed_20261016')
STATE=Path('/var/lib/payload_catch/ownership_20261016')
BASE=Path('/home/nvidia/payload_catch_deploy')
OLD='8accbc1eca8dcba99b747f953e33ffddee2ac55f16d09db4d48a6df77a4781dd'
OLD_UNITS=('microxrce-agent.service','mav-relay.service')
NEW_UNITS=('payload-catch-agent.service','payload-catch-mav-relay.service')
TIMER='payload-catch-ownership-rollback'


def run(args,**kwargs):return subprocess.run(args,check=True,timeout=kwargs.pop('timeout',25),**kwargs)
def output(args):return run(args,capture_output=True,text=True).stdout.strip()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write_json(path,data):
    temp=path.with_suffix(path.suffix+'.next');temp.write_text(json.dumps(data,indent=2)+'\n');temp.replace(path)


@contextlib.contextmanager
def locks():
    STATE.mkdir(parents=True,exist_ok=True)
    with (STATE/'operation.lock').open('a') as a,(BASE/'mission.lock').open('a') as b,(BASE/'operation.lock').open('a') as c:
        for stream in (a,b,c):fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield


def prepare(upload):
    assert not PREFIX.exists(),'preserve existing managed runtime for inspection'
    account=pwd.getpwnam('nvidia')
    PREFIX.mkdir(parents=True)
    for name in ('bin','lib','px4_msgs_ws/src'):(PREFIX/name).mkdir(parents=True)
    for p in (PREFIX/'px4_msgs_ws',PREFIX/'px4_msgs_ws/src'):os.chown(p,account.pw_uid,account.pw_gid)
    executable=Path('/usr/local/bin/MicroXRCEAgent')
    assert sha(executable)=='24c3f80b112867c22e5d31b7a94f38d3de38af80a5c843247346a72eb450c532'
    shutil.copy2(executable,PREFIX/'bin/MicroXRCEAgent')
    linked=output(['ldd',str(executable)])
    libraries=sorted(set(re.findall(r'/usr/local/lib/[A-Za-z0-9_.+-]+',linked)))
    assert libraries
    for library in libraries:shutil.copy2(library,PREFIX/'lib'/Path(library).name)
    for name in ('ground_status.py','ground_env.sh','manage_ownership.py'):
        shutil.copy2(upload/name,PREFIX/name);os.chmod(PREFIX/name,0o644)
    env=dict(os.environ,LD_LIBRARY_PATH=str(PREFIX/'lib'))
    check=run(['ldd',str(PREFIX/'bin/MicroXRCEAgent')],env=env,capture_output=True,text=True).stdout
    assert '/home/nvidia/' not in check and '/usr/local/lib/' not in check,check
    write_json(PREFIX/'agent_manifest.json',dict(binary=sha(PREFIX/'bin/MicroXRCEAgent'),
        libraries={p.name:sha(p) for p in (PREFIX/'lib').iterdir()}))
    print('Independent Agent binary/libs staged; source workspace ready for ordinary-user build.')


def seal():
    ws=PREFIX/'px4_msgs_ws'
    assert (ws/'install/setup.bash').is_file()
    assert output(['git','-c','safe.directory='+str(ws/'src/px4_msgs'),'-C',str(ws/'src/px4_msgs'),'rev-parse','HEAD'])=='392e831c1f659429ca83902e66820d7094591410'
    for p in (ws/'install').rglob('*'):
        if p.is_file() and p.suffix in ('.bash','.sh','.dsv','.pth'):
            assert '/home/nvidia/ros2_ws' not in p.read_text(errors='replace'),str(p)
    for directory,dirs,files in os.walk(PREFIX,followlinks=False):
        for p in [Path(directory),*(Path(directory)/n for n in dirs+files)]:
            os.chown(p,0,0,follow_symlinks=False)
            if not p.is_symlink():os.chmod(p,stat.S_IMODE(p.stat().st_mode)&~0o022)
    print('Runtime sealed root-owned; no old overlay in generated message environment.')


def guard():
    env={'HOME':'/root','PATH':'/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','LANG':'C.UTF-8'}
    run(['/bin/bash',str(PREFIX/'ground_env.sh')],env=env,timeout=15)


def snapshot(release):
    assert not (STATE/'before.json').exists(),'ownership change already recorded'
    assert (BASE/'current').resolve().name==OLD
    before=STATE/'before';before.mkdir()
    files=[Path('/etc/systemd/system')/n for n in OLD_UNITS]
    files.extend((BASE/'host/runtime.yaml',Path('/home/nvidia/.bashrc')))
    records=[]
    for i,p in enumerate(files):
        assert p.is_file() and not p.is_symlink(),str(p)
        target=before/str(i);shutil.copy2(p,target)
        st=p.stat();records.append(dict(path=str(p),backup=str(target),uid=st.st_uid,gid=st.st_gid,mode=stat.S_IMODE(st.st_mode)))
    for unit in NEW_UNITS:
        p=Path('/etc/systemd/system')/unit
        assert not p.exists() and not p.is_symlink(),'new unit already exists'
    units={name:dict(enabled=output(['systemctl','is-enabled',name]),active=output(['systemctl','is-active',name])) for name in OLD_UNITS}
    assert all(s['enabled']=='enabled' and s['active']=='active' for s in units.values())
    write_json(STATE/'before.json',dict(files=records,units=units,current=str((BASE/'current').resolve()),
        previous=str((BASE/'previous').resolve()),new_release=release,committed=False,phase='prepared'))


def link(name,target):
    p=BASE/name;temporary=BASE/(name+'.ownership-next')
    assert not temporary.exists() and not temporary.is_symlink()
    temporary.symlink_to(target,target_is_directory=True);temporary.replace(p)


def restore(force=False):
    p=STATE/'before.json'
    if not p.exists():return
    data=json.loads(p.read_text())
    if data['committed'] and not force:
        print('Committed; timed recovery has no work.');return
    if data['phase']=='rolled_back':return
    # Protect an active aircraft from an operator-invoked later rollback.
    if force:
        assert (BASE/'current').resolve().name==data['new_release'],'current release changed; inspect before restoring old state'
        guard()
    for unit in NEW_UNITS:
        subprocess.run(['systemctl','disable','--now',unit],check=False,timeout=20)
        file=Path('/etc/systemd/system')/unit
        if file.exists():file.unlink()
    for record in data['files']:
        file=Path(record['path'])
        if file.is_symlink():
            assert str(file.readlink())=='/dev/null';file.unlink()
        shutil.copy2(record['backup'],file)
        os.chown(file,record['uid'],record['gid']);os.chmod(file,record['mode'])
    link('current',data['current']);link('previous',data['previous'])
    run(['systemctl','daemon-reload'])
    for unit in OLD_UNITS:run(['systemctl','enable','--now',unit])
    data.update(phase='rolled_back',committed=False);write_json(p,data)
    print('Prior release, profile, shell and services restored; no flight/PWM commands.')


def switch(release):
    assert re.fullmatch(r'[0-9a-f]{64}',release)
    candidate=BASE/'releases'/release
    assert (candidate/'BUILT').is_file() and (candidate/'ROLE').read_text().strip()=='a'
    run(['/usr/bin/python3',str(candidate/'src/tools/fleet_deploy.py'),'verify',str(candidate/'src')])
    guard();snapshot(release)
    run(['systemd-run','--unit='+TIMER,'--on-active=180s','--timer-property=AccuracySec=1s',
         '/usr/bin/python3',str(PREFIX/'manage_ownership.py'),'rollback'])
    try:
        relay=candidate/'src/tools/mavlink_relay.py'
        shutil.copy2(relay,PREFIX/'mavlink_relay.py');os.chown(PREFIX/'mavlink_relay.py',0,0);os.chmod(PREFIX/'mavlink_relay.py',0o644)
        units={NEW_UNITS[0]:f'''[Unit]
Description=payload_catch A PX4 DDS Agent
After=network-online.target
Wants=network-online.target
Conflicts=microxrce-agent.service
StartLimitIntervalSec=0
[Service]
Type=simple
User=nvidia
Environment=ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH={PREFIX}/lib
ExecStart={PREFIX}/bin/MicroXRCEAgent udp4 -p 8888
Restart=always
RestartSec=2
[Install]
WantedBy=multi-user.target
''',NEW_UNITS[1]:f'''[Unit]
Description=payload_catch A FC to configured QGC relay
After=network-online.target
Wants=network-online.target
Conflicts=mav-relay.service
StartLimitIntervalSec=0
[Service]
Type=simple
User=nvidia
ExecStart=/usr/bin/python3 {PREFIX}/mavlink_relay.py --fc-ip 10.41.10.2 --gcs-ip 192.168.1.123 --port 14550
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
'''}
        run(['systemctl','disable','--now',*OLD_UNITS])
        for unit in OLD_UNITS:
            file=Path('/etc/systemd/system')/unit
            file.unlink()  # Exact regular unit already copied into the root-only backup.
        run(['systemctl','mask',*OLD_UNITS])
        for name,text in units.items():
            p=Path('/etc/systemd/system')/name;p.write_text(text);os.chmod(p,0o644)
        profile=BASE/'host/runtime.yaml'
        shutil.copy2(candidate/'src/config/jetson_a.runtime.yaml',profile)
        account=pwd.getpwnam('nvidia');os.chown(profile,account.pw_uid,account.pw_gid)
        rc=Path('/home/nvidia/.bashrc');text=rc.read_text()
        text=re.sub(r'(?m)^\s*export ROS_DOMAIN_ID=42\s*$',
                    '# Legacy ROS domain42 disabled: payload_catch owns this host.',text)
        text+='\n# payload_catch managed runtime (environment only; no task autostart)\nif [ -f "$HOME/payload_catch_deploy/current/src/deploy/runtime_env.sh" ]; then\n    source "$HOME/payload_catch_deploy/current/src/deploy/runtime_env.sh"\nfi\n'
        rc.write_text(text)
        link('previous',BASE/'releases'/OLD);link('current',candidate)
        run(['systemctl','daemon-reload']);run(['systemctl','enable','--now',*NEW_UNITS])
        data=json.loads((STATE/'before.json').read_text());data['phase']='applied';write_json(STATE/'before.json',data)
        print('Ownership applied; automatic rollback remains armed until verified commit.')
    except BaseException:
        restore();raise


def commit():
    data=json.loads((STATE/'before.json').read_text());assert data['phase']=='applied'
    guard()
    for name in NEW_UNITS:
        assert output(['systemctl','is-active',name])=='active'
        assert output(['systemctl','is-enabled',name])=='enabled'
    for name in OLD_UNITS:
        p=subprocess.run(['systemctl','is-enabled',name],capture_output=True,text=True)
        assert p.stdout.strip()=='masked'
        p=subprocess.run(['systemctl','is-active',name],capture_output=True,text=True)
        assert p.stdout.strip()=='inactive'
    assert (BASE/'current').resolve().name==data['new_release']
    data['committed']=True;data['phase']='committed';write_json(STATE/'before.json',data)
    run(['systemctl','stop',TIMER+'.timer'])
    print('Managed runtime committed; old service entrypoints masked; recovery backup retained.')


def main():
    assert os.geteuid()==0,'root required only for protected runtime/service management'
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('prepare','seal','switch','commit','rollback','restore','status'))
    p.add_argument('--upload');p.add_argument('--release');p.add_argument('--ground-confirmed',action='store_true')
    a=p.parse_args()
    with locks():
        if a.action=='prepare':prepare(Path(a.upload).resolve(strict=True))
        elif a.action=='seal':seal()
        elif a.action=='switch':switch(a.release)
        elif a.action=='commit':commit()
        elif a.action=='rollback':restore()
        elif a.action=='restore':
            assert a.ground_confirmed;restore(force=True)
        else:print((STATE/'before.json').read_text() if (STATE/'before.json').exists() else 'not switched')


if __name__=='__main__':main()
