from pathlib import Path
import json
import subprocess
ROOT=Path(__file__).resolve().parents[2]
PX=Path("/home/caolihao/drone_package_20260908/PX4-Autopilot-1.16")
B=PX/"build/px4_sitl_default"
out={}
cc=B/"compile_commands.json"
out["compile_commands_exists"]=cc.exists()
if cc.exists():
    data=json.loads(cc.read_text(encoding="utf-8"))
    match=[d for d in data if d["file"].endswith("/EKF/ekf.cpp")]
    out["ekf_compile"]=match
for label,args in [
    ("archive_symbols",["nm","-u",str(B/"src/modules/ekf2/EKF/libecl_EKF.a")]),
    ("libraries",["find",str(B),"-name","*.a"])]:
    p=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=15,shell=False)
    out[label]=dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
(Path(__file__).resolve().parent/"local_ekf_build_info.json").write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps({"compile":out.get("ekf_compile"),"libraries":out["libraries"]},indent=2))
