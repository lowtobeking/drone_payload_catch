"""Read-only static heading comparison. No FC input publishers or parameter writes."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleLocalPosition, EstimatorStatusFlags, VehicleAttitude
from payload_catch.mocap_odometry import load_config

ROOT=Path(__file__).resolve().parent
LOG=Path(sys.argv[1])
stop=False
def stopping(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,stopping)
signal.signal(signal.SIGTERM,stopping)
def clean(d):
    if isinstance(d,float) and not math.isfinite(d): return None
    if isinstance(d,dict): return {k:clean(v) for k,v in d.items()}
    if isinstance(d,(tuple,list)): return [clean(v) for v in d]
    if hasattr(d,"tolist"): return clean(d.tolist())
    return d
def save(name,data):
    (LOG/name).write_text(json.dumps(clean(data),indent=2,allow_nan=False),encoding="utf-8")
def angle(x):
    return math.atan2(math.sin(x),math.cos(x))
def circular(values):
    center=math.atan2(sum(math.sin(v) for v in values),sum(math.cos(v) for v in values))
    residual=[angle(v-center) for v in values]
    return dict(mean_deg=math.degrees(center),std_deg=math.degrees(float(np.std(residual))),
                span_deg=math.degrees(max(residual)-min(residual)))
def main():
    LOG.mkdir(parents=True,exist_ok=False)
    assert os.environ.get("ROS_DOMAIN_ID")=="0"
    assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n")=="1612425621719"
    cfg,transform=load_config(ROOT/"config/mocap_a.bridge.yaml")
    latest={}
    rows=[]
    pairs=[]
    child=None
    handle=None
    node=None
    result={"passed":False,"read_only_fcu":True,"flight_approval":False}
    try:
        rclpy.init(args=[])
        node=Node("a_mocap_heading_probe",start_parameter_services=False)
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        def ground():
            for k in ("status","land"):
                if k not in latest or time.monotonic()-latest[k][1]>1.5:
                    raise RuntimeError("fresh ground evidence missing: "+k)
            if latest["status"][0]["arming_state"]!=1 or not latest["land"][0]["landed"]:
                raise RuntimeError("requires disarmed/landed")
            if (latest["status"][0]["system_id"],latest["status"][0]["component_id"])!=(1,1):
                raise RuntimeError("wrong FC")
        def take(key,msg):
            d=json.loads(msg.data) if key=="raw" else {k:clean(getattr(msg,k)) for k in msg.get_fields_and_field_types()}
            now=time.monotonic()
            latest[key]=(d,now)
            rows.append(dict(kind=key,monotonic=now,data=d))
            if key=="raw" and "local" in latest and now-latest["local"][1]<.05:
                _,q=transform.convert(d["position"],d["quaternion_xyzw"])
                mocap=Rotation.from_quat(q).as_euler("xyz")
                fcu=latest["local"][0]
                pairs.append(dict(monotonic=now,mocap_rpy_deg=np.degrees(mocap).tolist(),
                    fcu_heading_deg=math.degrees(fcu["heading"]),
                    delta_deg=math.degrees(angle(fcu["heading"]-mocap[2])),
                    fcu_heading_variance=fcu["heading_var"],
                    px4_heading_good=fcu["heading_good_for_control"]))
        subs=[]
        for key,cls,topic in [
            ("status",VehicleStatus,"/fmu/out/vehicle_status_v1"),
            ("land",VehicleLandDetected,"/fmu/out/vehicle_land_detected"),
            ("local",VehicleLocalPosition,"/fmu/out/vehicle_local_position"),
            ("flags",EstimatorStatusFlags,"/fmu/out/estimator_status_flags"),
            ("attitude",VehicleAttitude,"/fmu/out/vehicle_attitude"),
            ("raw",String,"/payload/mocap/a/raw_sample")]:
            subs.append(node.create_subscription(cls,topic,lambda m,key=key:take(key,m),qos))
        end=time.monotonic()+5
        while time.monotonic()<end and not stop: rclpy.spin_once(node,timeout_sec=.01)
        ground()
        for suffix in ("vehicle_visual_odometry","vehicle_mocap_odometry","vehicle_command","trajectory_setpoint","offboard_control_mode"):
            if node.get_publishers_info_by_topic("/fmu/in/"+suffix):
                raise RuntimeError("unexpected active FC publisher: "+suffix)
        if node.get_publishers_info_by_topic("/payload/mocap/a/raw_sample"):
            raise RuntimeError("existing raw publisher")
        handle=(LOG/"source.log").open("wb")
        child=subprocess.Popen([sys.executable,"-m","payload_catch.mocap_source_node",
            "--reader",str(ROOT/"mocap_vrpn_reader"),"--bind-ip","192.168.1.4",
            "--seconds","19","--log-dir",str(LOG/"source")],cwd=ROOT,stdout=handle,stderr=subprocess.STDOUT,shell=False)
        save("task_processes.json",[dict(name="heading_raw_source",pid=child.pid)])
        end=time.monotonic()+14
        while time.monotonic()<end and not stop:
            rclpy.spin_once(node,timeout_sec=.01)
            ground()
            if child.poll() is not None: raise RuntimeError("source exited early")
        if stop: raise RuntimeError("interrupted")
        if len(pairs)<500: raise RuntimeError("insufficient paired heading samples")
        useful=pairs[len(pairs)//4:]
        delta=circular([math.radians(p["delta_deg"]) for p in useful])
        source=circular([math.radians(p["mocap_rpy_deg"][2]) for p in useful])
        fcu=circular([math.radians(p["fcu_heading_deg"]) for p in useful])
        raw=[r["data"] for r in rows if r["kind"]=="raw"]
        gaps=[b["receive_monotonic_us"]-a["receive_monotonic_us"] for a,b in zip(raw,raw[1:])]
        result.update(passed=delta["std_deg"]<1. and delta["span_deg"]<3.,
            pairs=len(pairs),sample_count=len(useful),mocap_yaw=source,px4_yaw=fcu,
            candidate_local_to_px4_yaw=delta,not_north_calibrated=True,
            max_native_gap_ms=max(gaps)/1000,
            last_position=raw[-1]["position"],last_q_xyzw=raw[-1]["quaternion_xyzw"],
            final={k:v[0] for k,v in latest.items() if k!="raw"})
        save("pairs.json",pairs)
    except BaseException as exc:
        result["error"]=repr(exc)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try: child.wait(timeout=7)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=3)
        if handle is not None: handle.close()
        save("process_cleanup.json",[] if child is None else [dict(pid=child.pid,exit_code=child.poll())])
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        with (LOG/"samples.jsonl").open("w",encoding="utf-8") as f:
            for row in rows: f.write(json.dumps(clean(row),allow_nan=False)+"\n")
        save("validation.json",result)
        print(json.dumps(clean(result),allow_nan=False),flush=True)
    return 0 if result["passed"] else 2
if __name__=="__main__": raise SystemExit(main())
