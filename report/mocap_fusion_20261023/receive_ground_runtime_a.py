"""Bounded real-FCU receive-time ground test. Never sends arm/flight/actuator commands."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (VehicleOdometry, VehicleStatus, VehicleLandDetected,
                         VehicleLocalPosition, EstimatorStatusFlags)
ROOT=Path(__file__).resolve().parent
LOG=Path(sys.argv[1])
stop=False
def stopping(*_):
    global stop
    stop=True
signal.signal(signal.SIGTERM,stopping)
signal.signal(signal.SIGINT,stopping)
def clean(d):
    if isinstance(d,float) and not math.isfinite(d): return None
    if isinstance(d,dict): return {k:clean(v) for k,v in d.items()}
    if isinstance(d,(tuple,list)): return [clean(v) for v in d]
    if hasattr(d,"tolist"): return clean(d.tolist())
    return d
def save(name,data):
    (LOG/name).write_text(json.dumps(clean(data),indent=2,allow_nan=False),encoding="utf-8")
def main():
    LOG.mkdir(parents=True,exist_ok=False)
    assert os.environ.get("ROS_DOMAIN_ID","0")=="0"
    assert os.environ.get("ROS_LOCALHOST_ONLY","0")=="0"
    assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n")=="1612425621719"
    handles=[]
    children=[]
    guard=None
    node=None
    rows=[]
    latest={}
    result={"passed":False,"flight_approval":False,"mode":"receive_time_ground"}
    def launch(name,argv):
        f=(LOG/(name+".log")).open("wb")
        handles.append(f)
        p=subprocess.Popen(argv,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,shell=False,
                           start_new_session=(name=="parameter_guard"))
        children.append((name,p))
        save("task_processes.json",[{"name":n,"pid":p.pid} for n,p in children])
        return p
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not stop:
            rclpy.spin_once(node,timeout_sec=.01)
        if stop: raise RuntimeError("test interrupted")
    try:
        rclpy.init(args=[])
        node=Node("a_receive_ground_capture",start_parameter_services=False)
        qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        def take(key,msg):
            data=json.loads(msg.data) if key in ("bridge","raw") else {
                k:clean(getattr(msg,k)) for k in msg.get_fields_and_field_types()}
            now=time.monotonic()
            latest[key]=(data,now)
            rows.append({"kind":key,"monotonic":now,"data":data})
        subs=[]
        for key,cls,topic in [
            ("status",VehicleStatus,"/fmu/out/vehicle_status_v1"),
            ("land",VehicleLandDetected,"/fmu/out/vehicle_land_detected"),
            ("local",VehicleLocalPosition,"/fmu/out/vehicle_local_position"),
            ("flags",EstimatorStatusFlags,"/fmu/out/estimator_status_flags"),
            ("fcu_odometry",VehicleOdometry,"/fmu/out/vehicle_odometry"),
            ("ev_input",VehicleOdometry,"/fmu/in/vehicle_visual_odometry"),
            ("bridge",String,"/payload/mocap/a/bridge_status"),
            ("raw",String,"/payload/mocap/a/raw_sample")]:
            subs.append(node.create_subscription(cls,topic,lambda m,key=key:take(key,m),qos))
        spin(6)
        def ground():
            for k in ("status","land"):
                if k not in latest or time.monotonic()-latest[k][1]>1.5:
                    raise RuntimeError("ground telemetry absent/stale: "+k)
            s=latest["status"][0]
            if (s["system_id"],s["component_id"],s["arming_state"])!=(1,1,1) or not latest["land"][0]["landed"]:
                raise RuntimeError("requires A disarmed and landed")
        ground()
        if node.get_publishers_info_by_topic("/fmu/in/vehicle_visual_odometry"):
            raise RuntimeError("existing EV publisher; refusing conflict")
        for suffix in ("vehicle_command","offboard_control_mode","trajectory_setpoint","actuator_motors","actuator_servos"):
            if node.get_publishers_info_by_topic("/fmu/in/"+suffix):
                raise RuntimeError("active flight input publisher: "+suffix)
        result["baseline"]={k:v[0] for k,v in latest.items()}
        guard=launch("parameter_guard",["sudo","-n","/usr/bin/env",
            "PYTHONPATH=/home/nvidia/.local/lib/python3.10/site-packages",
            "/usr/bin/python3",str(ROOT/"receive_parameter_guard_a.py"),str(LOG)])
        end=time.monotonic()+65
        while not (LOG/"backup_ready.json").exists() and time.monotonic()<end:
            spin(.1)
            ground()
            if guard.poll() is not None:
                raise RuntimeError("parameter backup/guard failed")
        if not (LOG/"backup_ready.json").exists():
            raise RuntimeError("backup readiness timeout")
        launch("source",[sys.executable,"-m","payload_catch.mocap_source_node",
            "--reader",str(ROOT/"mocap_vrpn_reader"),"--bind-ip","192.168.1.4",
            "--seconds","64","--log-dir",str(LOG/"source")])
        # Complete native connection and ROS discovery before starting the fail-closed bridge.
        spin(4)
        ground()
        raw_rows = [r for r in rows if r["kind"] == "raw"]
        if len(raw_rows) < 200:
            raise RuntimeError("source warmup insufficient")
        launch("bridge",[sys.executable,"-m","payload_catch.mocap_odometry_node",
            "--config",str(ROOT/"config/mocap_a.receive_ground.yaml"),"--send-to-px4",
            "--ground-receive-time","--seconds","65"])
        end=time.monotonic()+12
        while time.monotonic()<end:
            spin(.1)
            ground()
            b=latest.get("bridge",({},0))[0]
            if b.get("latched_fault"):
                raise RuntimeError("bridge latched: "+b["latched_fault"])
            if b.get("forwarded",0)>30:
                break
        if latest.get("bridge",({},0))[0].get("forwarded",0)<=30:
            raise RuntimeError("no healthy ground EV stream")
        (LOG/"enable.request").touch()
        end=time.monotonic()+6
        while not (LOG/"enabled.json").exists() and time.monotonic()<end:
            spin(.1)
            ground()
            if guard.poll() is not None:
                raise RuntimeError("parameter lease refused")
        if not (LOG/"enabled.json").exists(): raise RuntimeError("enable timeout")
        enabled_at=time.monotonic()
        result["enabled_at_monotonic"]=enabled_at
        end=enabled_at+27
        while time.monotonic()<end:
            spin(.1)
            ground()
            if any(p.poll() is not None for n,p in children):
                raise RuntimeError("ground test child exited early")
            b=latest.get("bridge",({},0))[0]
            if b.get("latched_fault"):
                raise RuntimeError("bridge latched: "+b["latched_fault"])
            if time.monotonic()-latest.get("bridge",({},0))[1]>1.5:
                raise RuntimeError("bridge status stale")
        during=[r for r in rows if r["monotonic"]>enabled_at+5]
        flags=[r for r in during if r["kind"]=="flags"]
        local=[r for r in during if r["kind"]=="local"]
        active=[r for r in flags if r["data"].get("cs_ev_pos") and r["data"].get("cs_ev_yaw")]
        valid=[r for r in local if all(r["data"].get(k) for k in ("xy_valid","v_xy_valid","heading_good_for_control"))]
        result.update(ev_both_active_samples=len(active),local_all_valid_samples=len(valid),
            ev_active_span_s=active[-1]["monotonic"]-active[0]["monotonic"] if len(active)>1 else 0,
            final={k:v[0] for k,v in latest.items()},
            passed=bool(len(active)>5 and len(valid)>5
                        and active[-1]["monotonic"]-active[0]["monotonic"]>10))
    except BaseException as exc:
        result["error"]=repr(exc)
    finally:
        (LOG/"release.request").touch()
        if guard is not None:
            try:
                guard.wait(timeout=35)
            except subprocess.TimeoutExpired:
                result["rollback_guard_pending"]=True
            if (LOG/"parameter_lease_result.json").exists():
                lease=json.loads((LOG/"parameter_lease_result.json").read_text(encoding="utf-8"))
                result["lease"]=lease
                if not lease.get("restored") or lease.get("sync_after")!=1:
                    result["passed"]=False
            else:
                result["passed"]=False
        for name,p in reversed(children):
            if name=="parameter_guard":
                continue
            if p.poll() is None:
                p.terminate()
                try: p.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=3)
        save("process_cleanup.json",[{"name":n,"pid":p.pid,"exit_code":p.poll()} for n,p in children])
        for f in handles: f.close()
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        with (LOG/"samples.jsonl").open("w",encoding="utf-8") as f:
            for row in rows: f.write(json.dumps(clean(row),allow_nan=False)+"\n")
        result["counts"]={k:sum(r["kind"]==k for r in rows) for k in latest}
        save("validation.json",result)
        print(json.dumps(clean(result),allow_nan=False),flush=True)
    return 0 if result.get("passed") else 2
if __name__=="__main__":
    raise SystemExit(main())
