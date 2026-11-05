"""Select external_local for A-only indoor operation; no PX4 writes or mission start."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
from probe_a import SSH
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[1]
m=json.loads((WORK/"manifest.json").read_text(encoding="utf-8"))
post=json.loads((WORK/"after.json").read_text(encoding="utf-8"))
local=ROOT/"config/a_solo.example.yaml"
data=local.read_bytes()
old=b"positioning: outdoor_gps  # external_local ONLY with real healthy external position + matching EKF"
new=b"positioning: external_local  # Indoor mocap, with healthy fused PX4 local position"
assert data.count(old)==1
(WORK/"a_solo.example.before_indoor.yaml").write_bytes(data)
local.write_bytes(data.replace(old,new,1))
code="""from pathlib import Path
import fcntl,hashlib,json,os,sys
base=Path('/home/nvidia/payload_catch_deploy')
stage=Path(%r)
assert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==%r
assert str((base/'current').resolve())==%r
sys.path.insert(0,str(base/'current/src'))
from payload_catch.solo_validation import load_config
with (base/'mission.lock').open('a') as lock:
 fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
 path=base/'a_solo.yaml'
 before=path.read_bytes()
 assert hashlib.sha256(before).hexdigest()==%r,'Site configuration changed; inspect rather than overwrite'
 previous=load_config(path)
 lines=before.splitlines(keepends=True)
 indices=[i for i,line in enumerate(lines) if line.startswith(b'positioning:')]
 assert len(indices)==1
 i=indices[0]
 assert b'outdoor_gps' in lines[i]
 lines[i]=lines[i].replace(b'outdoor_gps',b'external_local',1)
 after=b''.join(lines)
 backup=stage/'a_solo.before_indoor.yaml'
 assert not backup.exists()
 backup.write_bytes(before)
 temporary=stage/'a_solo.indoor.next'
 temporary.write_bytes(after)
 temporary.chmod(path.stat().st_mode & 0o777)
 updated=load_config(temporary)
 from dataclasses import asdict
 previous_fields=asdict(previous);updated_fields=asdict(updated)
 assert previous_fields.pop('positioning')=='outdoor_gps'
 assert updated_fields.pop('positioning')=='external_local'
 assert previous_fields==updated_fields and updated.gps is False
 assert path.read_bytes()==before
 os.replace(temporary,path)
 assert path.read_bytes()==after
 result=dict(positioning=updated.positioning,gps_required=updated.gps,height_m=updated.height_m,hover_s=updated.hover_s,
 site_checked=updated.site_checked,payload_removed=updated.payload_removed,
 backup=str(backup),before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(after).hexdigest(),
 changed_fields=['positioning'],px4_parameters_changed=False,mission_started=False)
 (stage/'indoor_config_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
 print(json.dumps(result,indent=2))
"""%(m["remote"],post["boot"],m["release"],post["files"]["a_solo.yaml"]["sha256"])
ast.parse(code)
(WORK/"configure_indoor_remote.py").write_text(code,encoding="utf-8")
p=subprocess.run(SSH+["/usr/bin/python3 -"],input=code.encode(),cwd=WORK,capture_output=True,timeout=35,shell=False)
(WORK/"indoor_config.stdout.log").write_bytes(p.stdout)
(WORK/"indoor_config.stderr.log").write_bytes(p.stderr)
if p.returncode:
 print(p.stdout.decode(errors="replace"))
 print(p.stderr.decode(errors="replace"))
 raise RuntimeError("Indoor site configuration failed")
result=json.loads(p.stdout)
(WORK/"indoor_config_result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
