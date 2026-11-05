from pathlib import Path
import ast,hashlib,json,os,subprocess,sys
stage=Path('/home/nvidia/payload_catch_a_deploy_20261101')
release=Path('/home/nvidia/payload_catch_deploy/releases/da8bcec5fd2a796727a771077a9fcbaf4a064056938035e54bb42bbb8be1cff4')
source=release/'src'
base=release.parent.parent
assert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()=='2e0322d9-53cb-40ae-a82b-abacfc391a3d'
assert str((base/'current').resolve())=='/home/nvidia/payload_catch_deploy/releases/b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37'
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
assert str((base/'previous').resolve())=='/home/nvidia/payload_catch_deploy/releases/b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37'
for path,digest in protected.items():
 assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
result.update(activated=True,current=str((base/'current').resolve()),previous=str((base/'previous').resolve()),mission_started=False)
(stage/'activation_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
