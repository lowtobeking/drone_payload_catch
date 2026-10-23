"""Bounded raw VRPN reception in isolated ROS domain; no PX4 publishers."""
from pathlib import Path
import json
import math
import os
import signal
import subprocess
import time
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

WORK=Path(__file__).resolve().parent
def main():
    assert os.environ["ROS_DOMAIN_ID"]=="97" and os.environ["ROS_LOCALHOST_ONLY"]=="1"
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE="1")
    logfile=WORK/"vrpn_client.log"
    rows={}
    subscriptions={}
    result={}
    rclpy.init()
    node=rclpy.create_node("payload_catch_raw_mocap_check")
    started=time.monotonic()
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    def receive(topic,msg):
        now=time.monotonic()
        row=rows.setdefault(topic,dict(count=0,first_rx=now,last_rx=now,invalid_samples=0))
        row["count"]+=1
        row["last_rx"]=now
        data=message_to_ordereddict(msg)
        row["last_sample"]=data
        if "pose" in data:
            pose=data["pose"]
            values=list(pose["position"].values())+list(pose["orientation"].values())
            if not all(math.isfinite(v) for v in values):row["invalid_samples"]+=1
    with logfile.open("w",encoding="utf-8") as log:
        child=subprocess.Popen(
            ["/opt/ros/humble/lib/vrpn_mocap/client_node","--ros-args","-r","__ns:=/vrpn_mocap",
             "-p","server:=192.168.1.111","-p","port:=3883",
             "-p","update_freq:=100.0","-p","refresh_freq:=1.0"],
            cwd=WORK,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print("VRPN_CHECK_PID="+str(child.pid)+" LOG="+str(logfile),flush=True)
        try:
            while time.monotonic()-started<22:
                for topic,types in node.get_topic_names_and_types():
                    if not topic.startswith("/vrpn_mocap/") or topic in subscriptions:continue
                    if not any(t.startswith("geometry_msgs/") for t in types):continue
                    cls=get_message(types[0])
                    subscriptions[topic]=node.create_subscription(cls,topic,lambda msg,t=topic:receive(t,msg),qos)
                rclpy.spin_once(node,timeout_sec=0.05)
                if child.poll() is not None:break
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    child.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
    now=time.monotonic()
    for row in rows.values():
        elapsed=row["last_rx"]-row["first_rx"]
        row["rate_hz"]=(row["count"]-1)/elapsed if elapsed>0 else 0
        row["last_rx_age_s"]=now-row["last_rx"]
    result=dict(server="192.168.1.111",port=3883,ros_domain=97,topics=rows,
                received_actual_samples=any(r["count"]>0 for r in rows.values()),
                px4_data_forwarded=False,flight_started=False,probe_process_exited=child.poll() is not None,
                probe_pid=child.pid,log=str(logfile))
    (WORK/"raw_vrpn_result.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2),flush=True)
    print(logfile.read_text(encoding="utf-8"),flush=True)
    return 0 if result["received_actual_samples"] else 2
if __name__=="__main__":
    raise SystemExit(main())
