from pathlib import Path
import fcntl,hashlib,json,os,sys
base=Path('/home/nvidia/payload_catch_deploy')
stage=Path('/home/nvidia/payload_catch_a_deploy_20261101')
assert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()=='2e0322d9-53cb-40ae-a82b-abacfc391a3d'
assert str((base/'current').resolve())=='/home/nvidia/payload_catch_deploy/releases/da8bcec5fd2a796727a771077a9fcbaf4a064056938035e54bb42bbb8be1cff4'
sys.path.insert(0,str(base/'current/src'))
from payload_catch.solo_validation import load_config
with (base/'mission.lock').open('a') as lock:
 fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
 path=base/'a_solo.yaml'
 before=path.read_bytes()
 assert hashlib.sha256(before).hexdigest()=='c614b86ca8ae0dea2a693b4caf316b1eaf0a3daf5b0af1679c25497d2ddbf5d1','Site configuration changed; inspect rather than overwrite'
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
