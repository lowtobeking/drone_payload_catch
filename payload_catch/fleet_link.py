"""Per-host role lock, topology, PX4 identities and mutual communication leases."""
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus
from .px4_iface import qos_in,topic_for
from .fleet_runtime import PeerLease


class FleetLink(Node):
    def __init__(self,**node_options):
        super().__init__('fleet_link',**node_options)
        for name,value in dict(role='',fingerprint='',lock_dir=str(Path.home()/'.cache/payload_catch')).items():
            self.declare_parameter(name,value)
        role=self.get_parameter('role').value
        self.lease=PeerLease(role,self.get_parameter('fingerprint').value,uuid.uuid4().hex)
        directory=Path(self.get_parameter('lock_dir').value);directory.mkdir(parents=True,exist_ok=True)
        self.lock=open(directory/('fleet-domain-'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock'),'a')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.status={};self.graph_ok=False;self.seq=0;self.graph_at=-1e9
        self.hello=self.create_publisher(String,'/fleet/hello_'+role,10)
        self.link=self.create_publisher(String,'/fleet/link_'+role,10)
        self.create_subscription(String,'/fleet/hello_'+self.lease.peer,self._peer,10)
        for i in (0,1):
            self.create_subscription(VehicleStatus,topic_for(i,'out/vehicle_status_v1'),
                lambda m,i=i:self._status(m,i),qos_in())
        self.timer=self.create_timer(.05,self._tick)

    def _status(self,m,i):
        self.status[i]=(time.monotonic(),m.system_id==i+1)

    def _peer(self,m):
        try:self.lease.accept(json.loads(m.data),time.time(),time.monotonic())
        except (ValueError,TypeError):self.lease.fault='malformed peer heartbeat'

    def _tick(self):
        wall,mono=time.time(),time.monotonic()
        if mono-self.graph_at>=.2:
            self.graph_at=mono
            names=Counter(n for n,namespace in self.get_node_names_and_namespaces() if namespace=='/')
            required=('a_node','b_node','safety_supervisor','release_guard','payload_servo','relnav_node',
                      'fleet_link_a','fleet_link_b')
            self.graph_ok=all(names[name]==1 for name in required)
            for topic in ('/safety/state','/payload/authorized_release','/payload/hardware_status','/payload/release_event',
                          '/drone_a/release_cmd','/drone_b/ready','/fleet/hello_a','/fleet/hello_b'):
                self.graph_ok=self.graph_ok and self.count_publishers(topic)==1
        identities=all(i in self.status and self.status[i][1] and mono-self.status[i][0]<=2 for i in (0,1))
        local_ok=self.graph_ok and identities;self.seq+=1
        self.hello.publish(String(data=json.dumps(dict(stamp=wall,seq=self.seq,role=self.lease.role,
            fingerprint=self.lease.fingerprint,boot=self.lease.boot,seen_peer_boot=self.lease.peer_boot,
            peer_rx_age_s=(mono-self.lease.received if self.lease.received is not None else None),local_ok=local_ok))))
        ready=self.lease.ready(wall,mono,local_ok)
        self.link.publish(String(data=json.dumps(dict(stamp=wall,ready=ready,
            reason=self.lease.fault or ('' if ready else 'waiting for topology / identities / bidirectional peer lease')))))

    def destroy_node(self):
        self.lock.close();return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=FleetLink();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
