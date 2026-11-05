"""Read only: compare forwarded odometry stamps with Jetson time and PX4 DDS sync."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from px4_msgs.msg import VehicleOdometry,TimesyncStatus

q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
rclpy.init()
n=Node("a_odom_time_readonly")
odom=[]
sync=[]
n.create_subscription(VehicleOdometry,"/fmu/in/vehicle_visual_odometry",
    lambda m:odom.append((time.time_ns()//1000,m)),q)
n.create_subscription(TimesyncStatus,"/fmu/out/timesync_status",
    lambda m:sync.append((time.time_ns()//1000,m)),q)
try:
    end=time.monotonic()+8
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
    a=odom[-1] if odom else None
    b=sync[-1] if sync else None
    print(json.dumps({"odom_count":len(odom),"sync_count":len(sync),
        "odom":({"age_us":a[0]-a[1].timestamp,"sample_lag_us":a[1].timestamp-a[1].timestamp_sample,
                 "position":list(a[1].position),"pose_frame":a[1].pose_frame,"quality":a[1].quality}
                if a else None),
        "sync":({"age_us":b[0]-b[1].timestamp,"remote_timestamp":b[1].remote_timestamp,
                 "estimated_offset":b[1].estimated_offset,"rtt_us":b[1].round_trip_time,
                 "source_protocol":b[1].source_protocol} if b else None)}, default=float))
finally:
    n.destroy_node()
    rclpy.shutdown()
