"""Read only: sample A PX4 EV fusion and local-position flags for 12 seconds."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleStatus

qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
rclpy.init()
node=Node("a_ev_readonly_check")
samples={"flags":[],"position":[],"status":[]}
node.create_subscription(EstimatorStatusFlags,"/fmu/out/estimator_status_flags",
    lambda m:samples["flags"].append((time.monotonic(),m)),qos)
node.create_subscription(VehicleLocalPosition,"/fmu/out/vehicle_local_position",
    lambda m:samples["position"].append((time.monotonic(),m)),qos)
node.create_subscription(VehicleStatus,"/fmu/out/vehicle_status_v1",
    lambda m:samples["status"].append((time.monotonic(),m)),qos)
try:
    end=time.monotonic()+12
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
    f=samples["flags"][-1][1] if samples["flags"] else None
    p=samples["position"][-1][1] if samples["position"] else None
    s=samples["status"][-1][1] if samples["status"] else None
    result=dict(counts={k:len(v) for k,v in samples.items()},
        ages={k:time.monotonic()-v[-1][0] if v else None for k,v in samples.items()},
        ev={k:getattr(f,k) for k in ("cs_ev_pos","cs_ev_yaw","cs_ev_hgt","cs_mag_hdg","cs_mag_3d")} if f else None,
        local={k:getattr(p,k) for k in ("xy_valid","z_valid","v_xy_valid","v_z_valid","heading_good_for_control","dead_reckoning")} if p else None,
        arming_state=s.arming_state if s else None)
    print(json.dumps(result))
finally:
    node.destroy_node()
    rclpy.shutdown()
