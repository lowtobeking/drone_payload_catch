"""Bounded fixed-world static/stream-loss ground capture, no flight or actuator output."""
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
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus,VehicleLandDetected,VehicleLocalPosition,VehicleOdometry,EstimatorStatusFlags
ROOT=Path(__file__).resolve().parent
LOG=Path(sys.argv[1])
stopping=False
def stop(*_):
    global stopping
    stopping=True
signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
def clean(d):
    if isinstance(d,float) and not math.isfinite(d): return None
    if isinstance(d,dict): return {k:clean(v) for k,v in d.items()}
    if isinstance(d,(list,tuple)): return [clean(v) for v in d]
    if hasattr(d,"tolist"): return clean(d.tolist())
    return d
def save(name,data): (LOG/name).write_text(json.dumps(clean(data),indent=2,allow_nan=False),encoding="utf-8")
def main():
    LOG.mkdir(parents=True,exist_ok=False)
    children=[]; handles=[]; node=None; guard=None; latest={}; rows=[]
    result=dict(passed=False,flight_approval=False,fixed_frame="aerolab_world_Y_X_minusZ_v1")
    def launch(name,args):
        f=(LOG/(name+".log")).open("wb"); handles.append(f)
        p=subprocess.Popen(args,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,shell=False,
                           start_new_session=(name=="guard"))
        children.append((name,p)); save("task_processes.json",[dict(name=n,pid=p.pid) for n,p in children])
        return p
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not stopping:
            rclpy.spin_once(node,timeout_sec=.01)
        if stopping: raise RuntimeError("interrupted")
    def ground():
        for key in ("status","land"):
            if key not in latest or time.monotonic()-latest[key][1]>1.5:
                raise RuntimeError("ground state missing/stale "+key)
        s=latest["status"][0]
        if (s["system_id"],s["component_id"],s["arming_state"])!=(1,1,1) or not latest["land"][0]["landed"]:
            raise RuntimeError("requires A disarmed/landed")
    try:
        assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n")=="1612425621719"
        assert os.environ.get("ROS_DOMAIN_ID")=="0"
        rclpy.init(args=[])
        def open_observer():
            nonlocal node
            node=Node("a_xyz_reference_ground_capture",start_parameter_services=False)
            qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            def take(key,msg):
                data=json.loads(msg.data) if key in ("raw","bridge") else {k:clean(getattr(msg,k)) for k in msg.get_fields_and_field_types()}
                now=time.monotonic()
                latest[key]=(data,now); rows.append(dict(kind=key,monotonic=now,data=data))
            node.owned_subscriptions=[]
            for key,cls,topic in [
                ("status",VehicleStatus,"/fmu/out/vehicle_status_v1"),("land",VehicleLandDetected,"/fmu/out/vehicle_land_detected"),
                ("local",VehicleLocalPosition,"/fmu/out/vehicle_local_position"),("flags",EstimatorStatusFlags,"/fmu/out/estimator_status_flags"),
                ("input",VehicleOdometry,"/fmu/in/vehicle_visual_odometry"),
                ("raw",String,"/payload/mocap/a/raw_sample"),("bridge",String,"/payload/mocap/a/bridge_status")]:
                node.owned_subscriptions.append(node.create_subscription(cls,topic,lambda m,key=key:take(key,m),qos))
        open_observer()
        spin(6); ground()
        for suffix in ("vehicle_visual_odometry","vehicle_mocap_odometry","vehicle_command","trajectory_setpoint","offboard_control_mode","actuator_motors","actuator_servos"):
            if node.get_publishers_info_by_topic("/fmu/in/"+suffix): raise RuntimeError("competing FC input "+suffix)
        guard=launch("guard",["sudo","-n","/usr/bin/env","PYTHONPATH=/home/nvidia/.local/lib/python3.10/site-packages",
            "/usr/bin/python3",str(ROOT/"fixed_guard_a.py"),str(LOG)])
        end=time.monotonic()+65
        while not (LOG/"backup_ready.json").exists() and time.monotonic()<end:
            spin(.1); ground()
            if guard.poll() is not None: raise RuntimeError("backup guard exited")
        if not (LOG/"backup_ready.json").exists(): raise RuntimeError("backup timeout")
        (LOG/"configure.request").touch()
        end=time.monotonic()+200
        while not (LOG/"enabled.json").exists() and time.monotonic()<end:
            spin(.1)  # FC reboot intentionally interrupts telemetry; guard verifies safe restart.
            if guard.poll() is not None: raise RuntimeError("configuration/restart guard exited")
        if not (LOG/"enabled.json").exists(): raise RuntimeError("configuration/restart timeout")
        # Rebuild only this task's participant: old subscriptions may not rematch
        # rebuilt PX4 DDS endpoints after the long FC restart.
        node.destroy_node()
        latest.clear()
        open_observer()
        end=time.monotonic()+25
        while time.monotonic()<end:
            spin(.1)
            if guard.poll() is not None: raise RuntimeError("guard exited waiting for post-reboot ROS")
            try:
                ground()
                break
            except RuntimeError:
                pass
        ground()
        source=launch("source",[sys.executable,"-m","payload_catch.mocap_source_node","--reader",str(ROOT/"mocap_vrpn_reader"),
            "--bind-ip","192.168.1.5","--seconds","120","--log-dir",str(LOG/"source")])
        spin(4); ground()
        if sum(r["kind"]=="raw" for r in rows)<200: raise RuntimeError("insufficient raw warmup")
        launch("bridge",[sys.executable,"-m","payload_catch.mocap_odometry_node",
            "--config",str(ROOT/"config/mocap_a.xyz_ground.yaml"),"--send-to-px4","--ground-receive-time",
            "--fixed-parameter-proof",str(LOG/"fixed_parameter_proof.json"),"--seconds","115"])
        end=time.monotonic()+10
        while time.monotonic()<end:
            spin(.1); ground()
            b=latest.get("bridge",({},0))[0]
            if b.get("latched_fault"): raise RuntimeError(b["latched_fault"])
            if b.get("forwarded",0)>30: break
        if latest.get("bridge",({},0))[0].get("forwarded",0)<=30: raise RuntimeError("no guarded EV stream")
        def healthy():
            ground()
            if any(p.poll() is not None for _,p in children): raise RuntimeError("test child exited")
            if latest.get("bridge",({},0))[0].get("latched_fault"): raise RuntimeError("bridge fault")
        def snapshot(label):
            now=time.monotonic()
            recent=[r for r in rows if now-4<=r["monotonic"]<=now]
            loc=[r["data"] for r in recent if r["kind"]=="local"]
            inp=[r["data"] for r in recent if r["kind"]=="input"]
            flags=[r["data"] for r in recent if r["kind"]=="flags"]
            if len(loc)<250 or len(inp)<60 or len(flags)<2: raise RuntimeError("insufficient settled data")
            required=("xy_valid","v_xy_valid","z_valid","v_z_valid","heading_good_for_control")
            if not all(all(d[k] for k in required) for d in loc): raise RuntimeError("invalid estimator state")
            if not all(all(d[k] for k in ("cs_ev_pos","cs_ev_yaw","cs_ev_hgt"))
                and not d["cs_mag_hdg"] and not d["cs_mag_3d"] for d in flags):
                raise RuntimeError("fusion flags invalid")
            import statistics
            def yaw(q):
                w,x,y,z=q
                return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            data=dict(label=label,monotonic=now,
                input_xyz=[statistics.mean(d["position"][i] for d in inp) for i in range(3)],
                px4_xyz=[statistics.mean(d[k] for d in loc) for k in ("x","y","z")],
                px4_z_span_m=max(d["z"] for d in loc)-min(d["z"] for d in loc),
                input_z_span_m=max(d["position"][2] for d in inp)-min(d["position"][2] for d in inp),
                input_yaw_deg=math.degrees(statistics.mean(yaw(d["q"]) for d in inp)),
                px4_yaw_deg=math.degrees(statistics.mean(d["heading"] for d in loc)),
                local_samples=len(loc),input_samples=len(inp),flags_samples=len(flags))
            save(label+".json",data)
            return data
        start=time.monotonic()
        while time.monotonic()-start<7:
            spin(.1);healthy()
        baseline=snapshot("baseline")
        save("manual_ready.json",dict(ready=True,baseline=baseline,
            max_interaction_seconds=96,instruction="lift slowly; hold steady 5 seconds; return after elevated captured; phases detected from fresh input"))
        save("live_status.json",dict(stage="await_lift",remaining_seconds=96))
        result["phases"]={"baseline":baseline}
        interaction_end=time.monotonic()+96
        from lift_plateau import stable_phase
        for phase in ("elevated","returned"):
            while True:
                spin(.1);healthy()
                now=time.monotonic()
                recent=[r for r in rows if r["kind"]=="input" and now-4<=r["monotonic"]<=now]
                zvalues=[r["data"]["position"][2] for r in recent]
                left=interaction_end-now
                if left<1: raise RuntimeError("manual interaction window expired")
                save("live_status.json",dict(stage="await_"+phase,remaining_seconds=left,
                    input_z=latest.get("input",({"position":[None,None,None]},0))[0]["position"][2],
                    px4_z=latest.get("local",({},0))[0].get("z")))
                full_window=(len(recent)>=60 and recent[-1]["monotonic"]-recent[0]["monotonic"]>=3.8)
                if full_window and stable_phase(zvalues,baseline["input_xyz"][2],phase):
                    sample=snapshot(phase)
                    result["phases"][phase]=sample
                    save("live_status.json",dict(stage=phase+"_captured",sample=sample,
                        remaining_seconds=interaction_end-time.monotonic()))
                    break
        base=result["phases"]["baseline"];high=result["phases"]["elevated"];back=result["phases"]["returned"]
        up_input=-(high["input_xyz"][2]-base["input_xyz"][2])
        up_px4=-(high["px4_xyz"][2]-base["px4_xyz"][2])
        return_input=back["input_xyz"][2]-base["input_xyz"][2]
        return_px4=back["px4_xyz"][2]-base["px4_xyz"][2]
        result["lift"]=dict(input_up_m=up_input,px4_up_m=up_px4,
            displacement_difference_m=abs(up_input-up_px4),
            return_input_from_baseline_m=return_input,return_px4_from_baseline_m=return_px4,
            return_difference_m=abs(return_input-return_px4))
        result["passed"]=(.15<up_input<1.2 and .15<up_px4<1.2
            and abs(up_input-up_px4)<.05 and abs(return_input)<.08 and abs(return_px4)<.08)
        save("movement_result.json",result)
    except BaseException as exc:
        result["error"]=repr(exc)
    finally:
        for name,p in reversed(children):
            if name=="guard" or p.poll() is not None: continue
            p.terminate()
            try: p.wait(timeout=7)
            except subprocess.TimeoutExpired: p.kill(); p.wait(timeout=3)
        (LOG/"release.request").touch()
        if guard is not None:
            try: guard.wait(timeout=650)
            except subprocess.TimeoutExpired: result["rollback_guard_pending"]=True
            if (LOG/"parameter_lease_result.json").exists():
                lease=json.loads((LOG/"parameter_lease_result.json").read_text(encoding="utf-8"))
                result["lease"]=lease
                if not lease.get("restored") or not lease.get("restore_restart_verified"): result["passed"]=False
            else: result["passed"]=False
        for name,p in reversed(children):
            if name=="guard": continue
            if p.poll() is None:
                p.terminate()
                try: p.wait(timeout=7)
                except subprocess.TimeoutExpired: p.kill(); p.wait(timeout=3)
        save("process_cleanup.json",[dict(name=n,pid=p.pid,exit_code=p.poll()) for n,p in children])
        for f in handles: f.close()
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        with (LOG/"samples.jsonl").open("w",encoding="utf-8") as f:
            for r in rows: f.write(json.dumps(clean(r),allow_nan=False)+"\n")
        result["counts"]={k:sum(r["kind"]==k for r in rows) for k in latest}
        save("validation.json",result); print(json.dumps(clean(result),allow_nan=False),flush=True)
    return 0 if result["passed"] else 2
if __name__=="__main__": raise SystemExit(main())
