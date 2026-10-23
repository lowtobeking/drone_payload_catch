"""Read-only network and bounded raw VRPN test; no ROS/flight output."""
from pathlib import Path
import json
import socket
import subprocess
import sys
serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
print("SERIAL",serial,flush=True)
if serial!="1612425621719":
    raise SystemExit("Device identity mismatch; stop")
work=Path("/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021")
work.mkdir(exist_ok=True)
def run(args,timeout=15):
    p=subprocess.run(args,cwd=work,capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=timeout,shell=False)
    print(json.dumps({"argv":args,"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}),flush=True)
    return p
run(["ip","-j","address"])
run(["ip","route"])
for peer in ["192.168.1.2","192.168.1.3","192.168.1.111"]:
    run(["ip","route","get",peer])
try:
    with socket.create_connection(("192.168.1.111",3883),timeout=3) as conn:
        conn.settimeout(3)
        print("VRPN_GREETING",repr(conn.recv(64)),flush=True)
except OSError as exc:
    print("VRPN_ERROR",str(exc),flush=True)
source=work/"planeA_probe.cpp"
source.write_text("#include <vrpn_Tracker.h>\n#include <vrpn_Connection.h>\n#include <chrono>\n#include <cmath>\n#include <iomanip>\n#include <iostream>\n#include <thread>\nstruct Samples {\n    unsigned long count=0, invalid=0, nonincreasing=0;\n    double stamp=0, p[3]={}, q[4]={};\n    std::chrono::steady_clock::time_point first,last;\n};\nvoid VRPN_CALLBACK pose_cb(void *data,const vrpn_TRACKERCB pose) {\n    auto &s=*static_cast<Samples*>(data);\n    auto now=std::chrono::steady_clock::now();\n    if(!s.count)s.first=now;\n    s.last=now;\n    double stamp=pose.msg_time.tv_sec+pose.msg_time.tv_usec*1e-6;\n    if(s.count && stamp<=s.stamp)++s.nonincreasing;\n    s.stamp=stamp;\n    bool valid=true;\n    for(int i=0;i<3;++i){s.p[i]=pose.pos[i];valid=valid && std::isfinite(s.p[i]);}\n    double norm=0;\n    for(int i=0;i<4;++i){s.q[i]=pose.quat[i];valid=valid && std::isfinite(s.q[i]);norm+=s.q[i]*s.q[i];}\n    if(!valid || std::abs(norm-1)>0.05)++s.invalid;\n    ++s.count;\n}\nint main(int argc,char **argv) {\n    const char *nic=argc>1?argv[1]:nullptr;\n    auto *conn=vrpn_get_connection_by_name(\"192.168.1.111:3883\",nullptr,nullptr,nullptr,nullptr,nic,true);\n    if(!conn)return 3;\n    Samples s;\n    {\n        vrpn_Tracker_Remote tracker(\"planeA@192.168.1.111:3883\",conn);\n        tracker.register_change_handler(&s,pose_cb);\n        auto start=std::chrono::steady_clock::now();\n        while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<10) {\n            tracker.mainloop();\n            std::this_thread::sleep_for(std::chrono::milliseconds(1));\n        }\n        auto end=std::chrono::steady_clock::now();\n        double span=s.count>1?std::chrono::duration<double>(s.last-s.first).count():0;\n        std::cout<<std::setprecision(15)<<\"{\\\"tracker\\\":\\\"planeA\\\",\\\"count\\\":\"<<s.count\n          <<\",\\\"invalid\\\":\"<<s.invalid<<\",\\\"nonincreasing_timestamps\\\":\"<<s.nonincreasing\n          <<\",\\\"connected\\\":\"<<(conn->connected()?\"true\":\"false\")\n          <<\",\\\"rate_hz\\\":\"<<(span>0?(s.count-1)/span:0)\n          <<\",\\\"age_s\\\":\"<<(s.count?std::chrono::duration<double>(end-s.last).count():-1)\n          <<\",\\\"position\\\":[\"<<s.p[0]<<\",\"<<s.p[1]<<\",\"<<s.p[2]\n          <<\"],\\\"quaternion_xyzw\\\":[\"<<s.q[0]<<\",\"<<s.q[1]<<\",\"<<s.q[2]<<\",\"<<s.q[3]\n          <<\"],\\\"forwarded_to_px4\\\":false}\"<<std::endl;\n    }\n    conn->removeReference();\n    return s.count>0?0:2;\n}\n",encoding="utf-8")
common=["/usr/bin/g++","-std=c++17","-I/opt/ros/humble/include"]
p=run(common+["-fsyntax-only",str(source)])
if p.returncode:
    raise SystemExit(p.returncode)
binary=work/"planeA_probe"
p=run(common+[str(source),"-L/opt/ros/humble/lib","-lvrpn","-lquat","-lpthread","-o",str(binary)],timeout=25)
if p.returncode:
    raise SystemExit(p.returncode)
route=run(["ip","-j","route","get","192.168.1.111"])
nic=json.loads(route.stdout)[0].get("prefsrc")
results=[]
for mode in ["default","explicit_wifi"]:
    if mode=="explicit_wifi" and not nic:
        continue
    p=run([str(binary)]+([nic] if mode=="explicit_wifi" else []),timeout=15)
    results.append({"mode":mode,"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr})
(work/"results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
print("PROBES_FINISHED",flush=True)
