"""Read-only guard for communication service replacement. No publishers."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from px4_msgs.msg import VehicleStatus,VehicleLandDetected

rclpy.init();node=Node('payload_catch_ownership_ground_guard',enable_rosout=False,start_parameter_services=False)
status=[];land=[]
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',
    lambda m:status.append((m.system_id,m.arming_state,m.timestamp)),q)
node.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',lambda m:land.append(m.landed),q)
try:
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    assert len(status)>=2 and all(s[0]==1 and s[1]==1 for s in status),'A identity/disarmed status missing'
    assert status[-1][2]>status[0][2] and land and all(land),'FC not confirmed landed/disarmed'
    print(json.dumps(dict(grounded=True,disarmed=True,samples=len(status),land_samples=len(land))))
finally:
    node.destroy_node();rclpy.shutdown()
