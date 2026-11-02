"""Add explicitly requested continuous, disarmed ground-session support."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/"report/mocap_continuous_20261026"
REPORT.mkdir(exist_ok=False)
def edit(relative,old,new):
    path=ROOT/relative
    value=path.read_text(encoding="utf-8")
    assert value.count(old)==1,(relative,old,value.count(old))
    backup=REPORT/(path.name+".before_continuous")
    if not backup.exists():
        backup.write_text(value,encoding="utf-8",newline="\n")
    value=value.replace(old,new,1)
    if path.suffix==".py":
        ast.parse(value,relative)
    path.write_text(value,encoding="utf-8",newline="\n")
edit("tools/mocap_vrpn_reader.cpp",
    'if (argc != 5 || std::string(argv[2]) != "planeA") {',
    'const bool continuous_ground = argc == 6 && std::string(argv[5]) == "--continuous-ground";\n    if ((argc != 5 && !continuous_ground) || std::string(argv[2]) != "planeA") {')
edit("tools/mocap_vrpn_reader.cpp",
    'if (!std::isfinite(seconds) || seconds < 1 || seconds > 120) return 2;',
    'if (!std::isfinite(seconds) || (continuous_ground ? seconds != 0 : (seconds < 1 || seconds > 120))) return 2;')
edit("tools/mocap_vrpn_reader.cpp",
    'while (running && std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds) {',
    'while (running && (continuous_ground || std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds)) {')
edit("payload_catch/mocap_source_node.py",
    'def run(reader, server, bind_ip, seconds, log_dir):',
    'def run(reader, server, bind_ip, seconds, log_dir, *, continuous_ground=False):')
edit("payload_catch/mocap_source_node.py",
    'str(seconds)], stdout=subprocess.PIPE, stderr=err, shell=False)',
    'str(seconds), *(["--continuous-ground"] if continuous_ground else [])], stdout=subprocess.PIPE, stderr=err, shell=False)')
edit("payload_catch/mocap_source_node.py",
    'while not stop and rclpy.ok() and time.monotonic() - start < seconds:',
    'while not stop and rclpy.ok() and (continuous_ground or time.monotonic() - start < seconds):')
edit("payload_catch/mocap_source_node.py",
    '    args = ap.parse_args()\n    if not 1 <= args.seconds <= 120:',
    '    ap.add_argument("--continuous-ground", action="store_true", help="explicit continuous raw ground source; requires --seconds 0")\n    args = ap.parse_args()\n    if args.continuous_ground and args.seconds != 0:\n        ap.error("--continuous-ground requires --seconds 0")\n    if not args.continuous_ground and not 1 <= args.seconds <= 120:')
edit("payload_catch/mocap_source_node.py",
    'run(args.reader, args.server, args.bind_ip, args.seconds, args.log_dir)',
    'run(args.reader, args.server, args.bind_ip, args.seconds, args.log_dir, continuous_ground=args.continuous_ground)')
edit("payload_catch/mocap_odometry_node.py",
    '    args = ap.parse_args()\n    if not 1 <= args.seconds <= 120:',
    '    ap.add_argument("--continuous-ground", action="store_true", help="continue until stopped, retaining disarmed ground gates; requires --seconds 0")\n    args = ap.parse_args()\n    if args.continuous_ground and (args.seconds != 0 or not args.send_to_px4 or not args.ground_receive_time or args.isolated_test):\n        ap.error("continuous ground session requires --seconds 0, --send-to-px4, --ground-receive-time; no isolated test")\n    if not args.continuous_ground and not 1 <= args.seconds <= 120:')
edit("payload_catch/mocap_odometry_node.py",
    'while not stopping and rclpy.ok() and time.monotonic() < end:',
    'while not stopping and rclpy.ok() and (args.continuous_ground or time.monotonic() < end):')
base=ROOT/"report/mocap_right_translation_20261026"
guard=(base/"fixed_guard_a.py").read_text(encoding="utf-8")
guard=guard.replace('ROOT/"parameters_backup_passive.json"','LOG/"parameters_backup_passive.json"')
guard=guard.replace('if backup.main()!=0:', 'if backup.main()!=0:')
# Backup implementation writes under its own stage ROOT, so each immutable stage is single-use.
old='''        end=time.monotonic()+150
        next_proof=0.
        while not stopping and not (LOG/"release.request").exists() and time.monotonic()<end:
            tap.receive()
'''
new='''        next_proof=0.
        while not stopping and not (LOG/"release.request").exists():
            tap.receive()
            heartbeat=json.loads((LOG/"session_heartbeat.json").read_text(encoding="utf-8"))
            now_us=time.monotonic_ns()//1000
            if (heartbeat.get("boot")!=Path("/proc/sys/kernel/random/boot_id").read_text().strip()
                    or type(heartbeat.get("monotonic_us")) is not int
                    or not 0<=now_us-heartbeat["monotonic_us"]<=3_000_000):
                raise RuntimeError("ground session supervisor heartbeat missing/stale")
'''
assert guard.count(old)==1
guard=guard.replace(old,new)
guard=guard.replace('max_lease_seconds=150','max_lease_seconds=None,supervisor_heartbeat_timeout_s=3,continuous_disarmed_ground=True')
# Preserve original backup location, immutable one-use stage guarantees no overwrite of previous runs.
guard=guard.replace('LOG/"parameters_backup_passive.json"','ROOT/"parameters_backup_passive.json"')
ast.parse(guard)
(ROOT/"tools/mocap_ground_guard_a.py").write_text(guard,encoding="utf-8",newline="\n")
runtime=(base/"fixed_runtime_a.py").read_text(encoding="utf-8")
runtime=runtime.replace('import json\n','import json\nfrom collections import deque\n',1)
runtime=runtime.replace('latest={}; rows=[]','latest={}; rows=deque(maxlen=12000)')
runtime=runtime.replace('str(ROOT/"fixed_guard_a.py")','str(ROOT/"mocap_ground_guard_a.py")')
runtime=runtime.replace('str(ROOT/"mocap_vrpn_reader")','str(ROOT/"mocap_vrpn_reader")')
runtime=runtime.replace('"--seconds","120","--log-dir"','"--seconds","0","--continuous-ground","--log-dir"')
runtime=runtime.replace('"--seconds","115"]','"--seconds","0","--continuous-ground"]')
runtime=runtime.replace('def save(name,data): (LOG/name).write_text(json.dumps(clean(data),indent=2,allow_nan=False),encoding="utf-8")','''def save(name,data):
    target=LOG/name
    temp=target.with_suffix(target.suffix+".tmp")
    temp.write_text(json.dumps(clean(data),indent=2,allow_nan=False),encoding="utf-8")
    temp.replace(target)''')
runtime=runtime.replace('    def spin(seconds):','''    def heartbeat():
        save("session_heartbeat.json",dict(boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            monotonic_us=time.monotonic_ns()//1000))
    def spin(seconds):''')
runtime=runtime.replace('            rclpy.spin_once(node,timeout_sec=.01)','''            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()-heartbeat.last_write>.2:
                heartbeat()
                heartbeat.last_write=time.monotonic()''')
runtime=runtime.replace('    try:\n        assert Path', '    heartbeat.last_write=0.\n    try:\n        assert Path',1)
runtime=runtime.replace('            b=latest.get("bridge",({},0))[0]','            b=latest.get("bridge",({},0))[0]')
start=runtime.index('        baseline=snapshot("baseline")')
end=runtime.index('    except BaseException as exc:',start)
runtime=runtime[:start]+'''        baseline=snapshot("baseline")
        result["phases"]={"baseline":baseline}
        result["continuous_ground_session"]=True
        started=time.monotonic()
        last_summary=0.
        while not stopping and not (LOG/"stop.request").exists():
            spin(.1);healthy()
            now=time.monotonic()
            if now-last_summary>=1:
                last_summary=now
                for key in ("input","local","flags","bridge"):
                    if key not in latest or now-latest[key][1]>1.5:
                        raise RuntimeError("session telemetry missing/stale "+key)
                f=latest["flags"][0]
                p=latest["local"][0]
                required=("xy_valid","v_xy_valid","z_valid","v_z_valid","heading_good_for_control")
                fused=all(f[k] for k in ("cs_ev_pos","cs_ev_hgt","cs_ev_yaw")) and not f["cs_mag_hdg"] and not f["cs_mag_3d"]
                ready=fused and all(p[k] for k in required)
                if not ready:
                    raise RuntimeError("XYZ/yaw fusion validity lost")
                save("live_status.json",dict(stage="fusing",ready=True,continuous_ground=True,
                    duration_s=now-started,flight_approval=False,armed=False,
                    input=latest["input"][0],local=p,flags=f,bridge=latest["bridge"][0],
                    task_processes=[dict(name=n,pid=p.pid) for n,p in children],
                    retained_sample_limit=12000,
                    monotonic_us=time.monotonic_ns()//1000))
        result["passed"]=True
''' +runtime[end:]
runtime=runtime.replace('    finally:\n        for name,p', '    finally:\n        save("live_status.json",dict(stage="restoring",ready=False,flight_approval=False,error=result.get("error")))\n        for name,p',1)
ast.parse(runtime)
(ROOT/"tools/mocap_ground_session_a.py").write_text(runtime,encoding="utf-8",newline="\n")
print("Prepared explicit continuous-ground source/bridge, bounded-memory supervisor and heartbeat rollback guard")
