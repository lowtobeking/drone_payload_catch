"""Validate staged code and atomically select it; no flight execution."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
from probe_a import SSH,run_probe
WORK=Path(__file__).resolve().parent
m=json.loads((WORK/"manifest.json").read_text(encoding="utf-8"))
before=json.loads((WORK/"before.json").read_text(encoding="utf-8"))
code="""from pathlib import Path
import ast,hashlib,json,os,subprocess,sys
stage=Path(%r)
release=Path(%r)
source=release/'src'
base=release.parent.parent
assert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==%r
assert str((base/'current').resolve())==%r
assert (release/'BUILT').is_file()
sys.path.insert(0,str(source))
import yaml
from payload_catch.solo_validation import load_config
cfg=load_config(base/'a_solo.yaml')
summary=dict(height_m=cfg.height_m,hover_s=cfg.hover_s,positioning=cfg.positioning,site_checked=cfg.site_checked,payload_removed=cfg.payload_removed)
(stage/'solo_config_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
for name in ['fixed_reference_20261023','mocap_z_20261023','mocap_fusion_20261023']:
 (source/'report'/name).mkdir(parents=True,exist_ok=True)
env=dict(os.environ,PYTHONPATH=str(source),PYTHONDONTWRITEBYTECODE='1')
tests=['test_mocap_transform.py','test_mocap_odometry.py','test_mocap_receive_ground.py','test_mocap_fixed_reference.py','test_mocap_height.py','test_mocap_flight_bridge.py','test_a_solo.py']
results=[]
for name in tests:
 p=source/'tools'/name
 ast.parse(p.read_text(encoding='utf-8'))
 r=subprocess.run(['/usr/bin/python3','-B',str(p)],cwd=source,env=env,capture_output=True,text=True,timeout=90)
 (stage/(name+'.stdout.log')).write_text(r.stdout,encoding='utf-8')
 (stage/(name+'.stderr.log')).write_text(r.stderr,encoding='utf-8')
 results.append(dict(test=name,exit=r.returncode,result=r.stderr.splitlines()[-6:]))
 if r.returncode:raise RuntimeError(json.dumps(results))
for name in ['run_a_solo.sh','run_a_mocap_flight.sh','run_a_solo_mocap.sh']:
 subprocess.run(['/bin/bash','-n',str(source/'deploy'/name)],check=True,timeout=5)
p=subprocess.run(['/usr/bin/python3',str(source/'tools/fleet_deploy.py'),'verify',str(source)],capture_output=True,text=True,timeout=25)
assert p.returncode==0,(p.stdout,p.stderr)
disarm=Path('/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0')
p=subprocess.run(['/bin/bash',str(disarm/'run_disarm.sh'),str(disarm)],capture_output=True,text=True,timeout=30)
(stage/'disarm.stdout.log').write_text(p.stdout,encoding='utf-8')
(stage/'disarm.stderr.log').write_text(p.stderr,encoding='utf-8')
assert p.returncode==0,(p.stdout,p.stderr)
fresh=json.loads(p.stdout.strip().splitlines()[-1])
assert fresh['passed'] and fresh['arming_state']==1 and fresh['landed'] is True,fresh
protected={}
for path in [base/'a_solo.yaml',base/'host/runtime.yaml',base/'host/managed/runtime.yaml']:
 if path.exists():protected[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
result=dict(tests=results,config=summary,disarmed=fresh,protected=protected)
(stage/'validation_before_activate.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
p=subprocess.run(['/usr/bin/python3',str(source/'tools/fleet_deploy.py'),'activate','--base',str(base),'--release',release.name,'--ground-confirmed'],capture_output=True,text=True,timeout=30)
(stage/'activate.stdout.log').write_text(p.stdout,encoding='utf-8')
(stage/'activate.stderr.log').write_text(p.stderr,encoding='utf-8')
assert p.returncode==0,(p.stdout,p.stderr)
assert (base/'current').resolve()==release
assert str((base/'previous').resolve())==%r
for path,digest in protected.items():
 assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
result.update(activated=True,current=str((base/'current').resolve()),previous=str((base/'previous').resolve()),mission_started=False)
(stage/'activation_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
"""%(m["remote"],m["release"],before["boot"],before["current"],before["current"])
ast.parse(code)
(WORK/"validate_activate_remote.py").write_text(code,encoding="utf-8")
p=subprocess.run(SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,capture_output=True,timeout=180,shell=False)
(WORK/"validation_activate.stdout.log").write_bytes(p.stdout)
(WORK/"validation_activate.stderr.log").write_bytes(p.stderr)
if p.returncode:
 print(p.stdout.decode(errors="replace"))
 print(p.stderr.decode(errors="replace"))
 raise RuntimeError("validation/activation failed: "+str(p.returncode))
result=json.loads(p.stdout)
(WORK/"activation_result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
m["activated"]=True
(WORK/"manifest.json").write_text(json.dumps(m,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
after=run_probe("after")
assert after["current"]==m["release"] and after["previous"]==before["current"]
assert after["boot"]==before["boot"]
assert after["services"]==before["services"],"Existing service state changed"
assert after["files"]==before["files"],"Existing configuration changed"
assert not after["task_processes"],"Unexpected task process"
(WORK/"post_verification.json").write_text(json.dumps(dict(passed=True,current=after["current"],previous=after["previous"],services_unchanged=True,configuration_unchanged=True,boot_unchanged=True,task_processes=[]),indent=2),encoding="utf-8")
print("POST-VERIFICATION PASSED; no mission started")
