"""Subscriber only: no publishers, parameter writes or vehicle commands."""
from pathlib import Path
from collections import Counter
import dataclasses
import json
import math
import sys
import time
root=Path("/home/nvidia/payload_catch_deploy/current/src")
sys.path.insert(0,str(root))
import rclpy
from rclpy.node import Node
from payload_catch.solo_validation_node import AHealth,KINDS,local_identity
from payload_catch.solo_validation import load_config,preflight_errors
from payload_catch.preflight_health import evaluate
from payload_catch.px4_iface import qos_in
from px4_msgs.msg import VehicleOdometry
def clean(v):
    if isinstance(v,float) and not math.isfinite(v):return None
    if isinstance(v,dict):return {str(k):clean(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [clean(x) for x in v]
    if hasattr(v,"tolist"):return clean(v.tolist())
    return v
def fields(msg):
    return {key:clean(getattr(msg,key)) for key in msg.get_fields_and_field_types()}
rclpy.init()
node=Node("a_readonly_postqgc_20261021")
health=AHealth(node)
counts=Counter()
latest={}
received={}
def receive(key,msg):
    counts[key]+=1
    latest[key]=msg
    received[key]=time.monotonic()
subscriptions=[node.create_subscription(cls,"/fmu/out/"+suffix,lambda m,k=k:receive(k,m),qos_in()) for k,(cls,suffix) in KINDS.items()]
for suffix in ["vehicle_odometry","vehicle_visual_odometry"]:
    subscriptions.append(node.create_subscription(VehicleOdometry,"/fmu/out/"+suffix,lambda m,k=suffix:receive(k,m),qos_in()))
try:
    start=time.monotonic()
    while time.monotonic()-start<8:
        rclpy.spin_once(node,timeout_sec=.03)
    cfg=load_config("/home/nvidia/payload_catch_deploy/a_solo.yaml")
    snap,telemetry=health.observe()
    control_topics=["/fmu/in/"+s for s in ["vehicle_command","offboard_control_mode","trajectory_setpoint","vehicle_visual_odometry","vehicle_mocap_odometry","vehicle_attitude_setpoint","vehicle_rates_setpoint","actuator_motors"]]
    publishers={topic:[{"node":info.node_name,"namespace":info.node_namespace} for info in node.get_publishers_info_by_topic(topic)] for topic in control_topics}
    result={"read_only":True,"identity":local_identity(),"sample_seconds":8,"counts":dict(counts),
        "receive_age_s":{k:time.monotonic()-v for k,v in received.items()},
        "raw":{k:fields(v) for k,v in latest.items()},
        "snapshot":dataclasses.asdict(snap),"telemetry":telemetry,
        "site_errors":cfg.site_errors(local_identity()),
        "configured_positioning":cfg.positioning,
        "configured_gate_errors":preflight_errors(snap,telemetry,cfg),
        "external_local_health_checks":evaluate(snap,require_gps=False,require_rc=True),
        "publishers":publishers,"nodes":node.get_node_names_and_namespaces(),
        "topics":node.get_topic_names_and_types()}
    output=json.dumps(clean(result),ensure_ascii=False,indent=2,allow_nan=False)
    Path("/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021/flight_health_postqgc.json").write_text(output,encoding="utf-8")
    print(output,flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
