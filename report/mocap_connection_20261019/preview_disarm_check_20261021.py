"""Subscriber only, PX4 1.16 versioned status topic; no vehicle command."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLandDetected
rclpy.init()
node=Node("a_mocap_readonly_disarm_check",start_parameter_services=False)
latest={}
received={}
def receive(key,msg):
    latest[key]=msg
    received[key]=time.monotonic()
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
subs=[
    node.create_subscription(VehicleStatus,"/fmu/out/vehicle_status_v1",lambda m:receive("status",m),qos),
    node.create_subscription(VehicleLandDetected,"/fmu/out/vehicle_land_detected",lambda m:receive("land",m),qos)]
try:
    start=time.monotonic()
    while time.monotonic()-start<8:
        rclpy.spin_once(node,timeout_sec=.05)
    s,l=latest.get("status"),latest.get("land")
    result=dict(arming_state=None if s is None else s.arming_state,
                landed=None if l is None else l.landed,
                age_s={k:time.monotonic()-v for k,v in received.items()},
                disarmed_constant=VehicleStatus.ARMING_STATE_DISARMED)
    result["passed"]=(s is not None and l is not None and s.arming_state==VehicleStatus.ARMING_STATE_DISARMED
                      and l.landed and all(age<2 for age in result["age_s"].values()))
    print(json.dumps(result),flush=True)
    if not result["passed"]:raise SystemExit(2)
finally:
    node.destroy_node()
    rclpy.shutdown()
