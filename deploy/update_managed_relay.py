"""Update only the project's relay/version, retaining the original ownership rollback."""
from pathlib import Path
import argparse
import importlib.util
import json
import os
import pwd
import re
import shutil
import subprocess

prefix=Path('/opt/payload_catch/managed_20261016')
spec=importlib.util.spec_from_file_location('ownership_manager',prefix/'manage_ownership.py')
manager=importlib.util.module_from_spec(spec);spec.loader.exec_module(manager)
assert os.geteuid()==0
p=argparse.ArgumentParser();p.add_argument('--release',required=True);a=p.parse_args()
assert re.fullmatch(r'[0-9a-f]{64}',a.release)
base=manager.BASE
first='a83087ff6afdb251861568c19702343fa666f0dd79c13b1aef234e7221a92295'
candidate=base/'releases'/a.release
with manager.locks():
    assert (base/'current').resolve().name==first
    assert (base/'previous').resolve().name==manager.OLD
    state=json.loads((manager.STATE/'before.json').read_text())
    assert state['new_release']==first and state['committed']
    manager.guard()
    assert (candidate/'BUILT').is_file()
    subprocess.run(['/usr/bin/python3',str(candidate/'src/tools/fleet_deploy.py'),'verify',str(candidate/'src')],check=True)
    backup=manager.STATE/'relay_before_nonblocking_fix';backup.mkdir()
    relay=prefix/'mavlink_relay.py';profile=base/'host/runtime.yaml'
    for path in (relay,profile):shutil.copy2(path,backup/path.name)
    manager.write_json(backup/'ownership_state.json',state)
    try:
        temporary=prefix/'mavlink_relay.py.next'
        shutil.copy2(candidate/'src/tools/mavlink_relay.py',temporary);os.chown(temporary,0,0);os.chmod(temporary,0o644)
        temporary.replace(relay)
        temporary=profile.with_suffix('.yaml.next')
        shutil.copy2(candidate/'src/config/jetson_a.runtime.yaml',temporary)
        account=pwd.getpwnam('nvidia');os.chown(temporary,account.pw_uid,account.pw_gid)
        temporary.replace(profile)
        manager.link('current',candidate)
        state['intermediate_managed_release']=first;state['new_release']=a.release
        manager.write_json(manager.STATE/'before.json',state)
        subprocess.run(['systemctl','restart','payload-catch-mav-relay.service'],check=True,timeout=20)
    except BaseException:
        shutil.copy2(backup/'mavlink_relay.py',relay);os.chown(relay,0,0)
        shutil.copy2(backup/'runtime.yaml',profile)
        manager.link('current',base/'releases'/first)
        manager.write_json(manager.STATE/'before.json',json.loads((backup/'ownership_state.json').read_text()))
        subprocess.run(['systemctl','stop','payload-catch-mav-relay.service'],check=False,timeout=20)
        raise
print('Nonblocking relay installed; Agent was not restarted; old services remain masked.')
