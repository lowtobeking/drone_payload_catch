"""Compile against existing PX41.16 ECL archive; syntax-check before linking and running."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import sys
WORK=Path(__file__).resolve().parent
PX=Path("/home/caolihao/drone_package_20260908/PX4-Autopilot-1.16")
B=PX/"build/px4_sitl_default"
data=json.loads((B/"compile_commands.json").read_text(encoding="utf-8"))
entry=next(d for d in data if d["file"].endswith("/EKF/ekf.cpp") and "ecl_EKF.dir" in d["command"])
tokens=shlex.split(entry["command"])
flags=[]
skip=False
for token in tokens[1:]:
    if skip:
        skip=False
        continue
    if token=="-o":
        skip=True
        continue
    if token=="-c" or token==entry["file"]: continue
    flags.append(token)
src=WORK/"ekf_fixed_reference.cpp"
exe=WORK/"ekf_fixed_reference"
lib=B/"src/modules/ekf2/EKF/libecl_EKF.a"
extras=[B/"src/lib/geo/libgeo.a",B/"src/lib/world_magnetic_model/libworld_magnetic_model.a",
        B/"src/lib/lat_lon_alt/liblat_lon_alt.a"]
objects=[]
for folder in ("bias_estimator","output_predictor","yaw_estimator"):
    objects.extend((B/"src/modules/ekf2/EKF"/folder).rglob("*.o"))
assert objects, "existing EKF submodule objects missing"
commands=[
    ("syntax",[tokens[0],*flags,"-I"+str(PX/"src/modules/ekf2"),"-fsyntax-only",str(src)]),
    ("link",[tokens[0],*flags,"-I"+str(PX/"src/modules/ekf2"),str(src),"-Wl,--start-group",str(lib),*map(str,objects),
             *map(str,extras),"-Wl,--end-group","-Wl,--gc-sections","-o",str(exe)]),
    ("run",[str(exe)])]
result={"px4_source":str(PX),"archive_sha256":hashlib.sha256(lib.read_bytes()).hexdigest(),"steps":[]}
for label,argv in commands:
    p=subprocess.run(argv,cwd=WORK,timeout=50,capture_output=True,text=True,
                     encoding="utf-8",errors="replace",shell=False)
    (WORK/(label+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (WORK/(label+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    result["steps"].append(dict(name=label,argv=argv,exit=p.returncode))
    print(label,p.returncode,p.stdout,p.stderr,flush=True)
    if p.returncode: break
result["passed"]=len(result["steps"])==3 and all(d["exit"]==0 for d in result["steps"])
(WORK/"local_ekf_validation.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
raise SystemExit(0 if result["passed"] else 2)
