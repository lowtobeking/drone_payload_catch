#!/usr/bin/env python3
"""Read-only commissioning probe. Never publishes commands or enables hardware."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=float,default=10)
    args=p.parse_args()
    if not 1<=args.seconds<=60:p.error('seconds must be 1..60')
    rclpy.init();node=Node('fleet_readonly_probe');state={};times={'a':[],'b':[]};stamps={'a':[],'b':[]}
    def receive(m,role):
        try:
            d=json.loads(m.data);state[role]=(time.monotonic(),d)
            times[role].append(time.monotonic());stamps[role].append(time.time()-d['stamp'])
        except (ValueError,KeyError,TypeError):pass
    for role in ('a','b'):node.create_subscription(String,'/fleet/link_'+role,lambda m,r=role:receive(m,r),10)
    deadline=time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        result={}
        for role in ('a','b'):
            when,data=state.get(role,(0,{}));samples=times[role]
            gaps=[b-a for a,b in zip(samples,samples[1:])]
            result[role]=dict(samples=len(samples),fresh=time.monotonic()-when<=.3,
                ready=data.get('ready') is True,reason=data.get('reason','no messages'),
                max_gap_s=max(gaps,default=None),max_clock_plus_network_age_s=max(stamps[role],default=None))
        print(json.dumps(result,indent=2))
        return 0 if all(d['fresh'] and d['ready'] for d in result.values()) else 1
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
