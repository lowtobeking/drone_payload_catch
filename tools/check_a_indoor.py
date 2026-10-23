#!/usr/bin/env python3
"""Initialize installed A-side nodes in an ISOLATED network; never drive hardware."""
import json,os,time
from pathlib import Path

def main():
    if (not os.environ.get('PAYLOAD_CATCH_HOST_NETNS')
            or os.readlink('/proc/self/ns/net')==os.environ['PAYLOAD_CATCH_HOST_NETNS']
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise RuntimeError('requires separate network namespace and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleCommand,OffboardControlMode,TrajectorySetpoint
    from payload_catch.a_node import ANode
    from payload_catch.release_guard import ReleaseGuard
    from payload_catch.payload_servo_node import PayloadServo
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from payload_catch.payload_servo import ServoConfig
    from payload_catch.fleet_runtime import PeerLease
    ServoConfig().validate()
    rclpy.init(args=['--ros-args','-p','auto_arm:=false','-p','preflight_report:=/nonexistent/payload-catch-indoor-no-flight.json',
        '-p','require_fleet_link:=true','-p','release_lead:=1.7','-p','release_delay_s:=1.5',
        '-p','enabled:=false','-p','calibrated:=false','-p','delay_reference:=operator_direct'])
    executor=SingleThreadedExecutor()
    a=ANode();guard=ReleaseGuard();servo=PayloadServo();probe=Node('a_indoor_probe')
    nodes=[a,guard,servo,probe]
    counts=dict(vehicle_command=0,offboard=0,setpoint=0,authorized_release=0)
    statuses=[]
    def observed(name):
        def callback(msg):counts[name]+=1
        return callback
    monitor_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
    for kind,topic,name in ((VehicleCommand,'/fmu/in/vehicle_command','vehicle_command'),
        (OffboardControlMode,'/fmu/in/offboard_control_mode','offboard'),
        (TrajectorySetpoint,'/fmu/in/trajectory_setpoint','setpoint')):
        probe.create_subscription(kind,topic,observed(name),monitor_qos)
    probe.create_subscription(String,'/payload/authorized_release',observed('authorized_release'),10)
    probe.create_subscription(String,'/payload/hardware_status',lambda m:statuses.append(json.loads(m.data)),10)
    try:
        for node in nodes:executor.add_node(node)
        end=time.monotonic()+5
        while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        assert all(pub.get_subscription_count()>0 for pub in (a.pub_cmd,a.pub_mode,a.pub_sp)), 'monitor must match each command publisher'
        assert all(v==0 for v in counts.values()),counts
        assert not a.mission_allowed() and not a.auto_arm
        assert servo.core.state=='DISABLED' and not servo.backend.pulses
        assert statuses and all(s['ready'] is False for s in statuses)
        result=dict(passed=True,network_isolated=True,auto_arm=False,hardware_output=False,
                    flight_gate_allowed=False,status_samples=len(statuses),counts=counts)
        print('A_INDOOR_RESULT='+json.dumps(result,sort_keys=True))
    finally:
        for node in nodes:
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()

if __name__=='__main__':main()
