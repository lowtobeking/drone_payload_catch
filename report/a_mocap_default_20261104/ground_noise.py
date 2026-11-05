import json
import math
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
rclpy.init()
node=Node('a_ground_noise_readonly')
rows=[]
status=[]
node.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',lambda m:rows.append((time.monotonic(),m.x,m.y,m.z,m.vx,m.vy,m.vz,m.xy_valid,m.z_valid,m.v_xy_valid,m.v_z_valid,m.dead_reckoning)),qos)
node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',lambda m:status.append(m.arming_state),qos)
try:
    end=time.monotonic()+20
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.03)
    values=lambda i:[r[i] for r in rows]
    result={'count':len(rows),'armed_states':sorted(set(status)),'all_valid':all(all(r[i] for i in (7,8,9,10)) and not r[11] for r in rows),'axes':{name:{'span':max(values(i))-min(values(i)),'std':statistics.pstdev(values(i))} for i,name in enumerate(('x','y','z','vx','vy','vz'),1)},'speed_p95':sorted(math.sqrt(sum(r[i]**2 for i in (4,5,6))) for r in rows)[int(len(rows)*.95)]}
    print(json.dumps(result))
finally:
    node.destroy_node()
    rclpy.shutdown()
