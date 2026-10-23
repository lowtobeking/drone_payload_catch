"""planeA-only VRPN source for isolated preview. Not a PX4 sender."""
import argparse
import json
import os
import selectors
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

def main():
    if os.environ.get("ROS_DOMAIN_ID")!="191" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
        raise SystemExit("preview source requires localhost domain 191")
    ap=argparse.ArgumentParser()
    ap.add_argument("--reader",required=True)
    ap.add_argument("--nic",required=True)
    args=ap.parse_args()
    rclpy.init()
    node=Node("a_planeA_native_preview_source",start_parameter_services=False)
    pub=node.create_publisher(PoseStamped,"/vrpn_mocap/planeA/pose",5)
    child=None
    selector=selectors.DefaultSelector()
    buffer=b""
    accepted=rejected=0
    last_stamp=0
    previous_report=time.monotonic()
    try:
        child=subprocess.Popen([args.reader,args.nic],stdout=subprocess.PIPE,stderr=None,shell=False)
        print(json.dumps(dict(reader_pid=child.pid,source="planeA-only",server="192.168.1.111:3883",local_timestamps=True,server_timing_verified=False)),flush=True)
        os.set_blocking(child.stdout.fileno(),False)
        selector.register(child.stdout,selectors.EVENT_READ)
        start=time.monotonic()
        while rclpy.ok() and time.monotonic()-start<26:
            for _,_ in selector.select(timeout=.01):
                chunk=os.read(child.stdout.fileno(),65536)
                if not chunk:
                    raise RuntimeError("native reader exited: "+str(child.poll()))
                buffer+=chunk
                if len(buffer)>262144:raise RuntimeError("native reader buffer overflow")
                lines=buffer.split(b"\n")
                buffer=lines.pop()
                latest=None
                for line in lines:
                    try:
                        data=json.loads(line)
                        stamp=int(data["server_stamp_us"])
                        if data["tracker"]!="planeA" or data["sensor"]!=0 or stamp<=last_stamp:
                            raise ValueError("wrong source or nonincreasing source timestamp")
                        if len(data["position"])!=3 or len(data["quaternion_xyzw"])!=4:
                            raise ValueError("wrong dimensions")
                        last_stamp=stamp
                        latest=data
                    except (ValueError,KeyError,TypeError):
                        rejected+=1
                if latest is not None:
                    msg=PoseStamped()
                    msg.header.frame_id="world"
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,latest["position"])
                    msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=map(float,latest["quaternion_xyzw"])
                    pub.publish(msg)
                    accepted+=1
            rclpy.spin_once(node,timeout_sec=0.)
            if time.monotonic()-previous_report>=1:
                print(json.dumps(dict(published=accepted,rejected=rejected,last_server_stamp_us=last_stamp,server_timing_verified=False)),flush=True)
                previous_report=time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        selector.close()
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=3)
            child.stdout.close()
            print(json.dumps(dict(reader_pid=child.pid,reader_exit=child.returncode,published=accepted,rejected=rejected)),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=="__main__":
    main()
