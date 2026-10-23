#!/usr/bin/env python3
"""Fresh, quality-gated RTK relative navigation; never retimestamp cached fixes."""
from __future__ import annotations
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, Bool
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from .px4_iface import qos_in
from .relnav import geodetic_to_ned, compensate_lever_arm, a_state_for_b, VelEstimator
from .safety_runtime import SourceWatch


def quat_to_rpy(q):
    w,x,y,z=q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


class RelNavNode(Node):
    def __init__(self):
        super().__init__('relnav_node')
        params=dict(source='rtk',publish_hz=50.,out_topic='/drone_a/state',
            a_rtk_topic='/rtk/a',b_rtk_topic='/rtk/b',rtk_type='navsatfix',use_geodetic=True,
            origin_lat=0.,origin_lon=0.,origin_alt=0.,a_lever=[0.,0.,0.],b_lever=[0.,0.,0.],
            a_lpos_topic='/fmu/out/vehicle_local_position',b_lpos_topic='/px4_1/fmu/out/vehicle_local_position',
            a_att_topic='/fmu/out/vehicle_attitude',b_att_topic='/px4_1/fmu/out/vehicle_attitude',
            a_world_offset=[0.,0.,0.],b_world_offset=[0.,0.,0.],source_timeout_s=.5,
            source_sync_s=.10,relative_sigma_max=.03)
        for k,v in params.items():self.declare_parameter(k,v)
        def value(k):return self.get_parameter(k).value
        self.source=str(value('source'));self.rtk_type=str(value('rtk_type'))
        if self.source not in ('rtk','px4'):raise ValueError('source must be rtk or px4')
        self.use_geo=bool(value('use_geodetic'))
        self.a_lever=np.asarray(value('a_lever'),float);self.b_lever=np.asarray(value('b_lever'),float)
        self.a_off=np.asarray(value('a_world_offset'),float);self.b_off=np.asarray(value('b_world_offset'),float)
        self.sync=float(value('source_sync_s'));self.sigma_max=float(value('relative_sigma_max'))
        if not (0<self.sync<=.5 and 0<self.sigma_max<=.1):raise ValueError('invalid navigation quality bounds')
        self.watch={k:SourceWatch(float(value('source_timeout_s'))) for k in ('a','b','a_pos','b_pos','a_att','b_att')}
        self.fix={};self.sigma={};self.pose={};self.att={};self.px4_stamp={}
        origin=tuple(float(value(k)) for k in ('origin_lat','origin_lon','origin_alt'))
        self.origin=origin if any(origin) else None
        self.vel=VelEstimator();self.last_source=None
        self.pub=self.create_publisher(Float64MultiArray,str(value('out_topic')),10)
        self.valid_pub=self.create_publisher(Bool,'/relnav/valid',10)
        # B PX4 and both attitudes are required in RTK mode as well.
        for who in ('a','b'):
            self.create_subscription(VehicleLocalPosition,str(value(who+'_lpos_topic')),
                lambda m,who=who:self._on_px4(m,who),qos_in())
            self.create_subscription(VehicleAttitude,str(value(who+'_att_topic')),
                lambda m,who=who:self._on_att(m,who),qos_in())
            if self.source=='rtk':
                self.create_subscription(NavSatFix if self.rtk_type=='navsatfix' else Float64MultiArray,
                    str(value(who+'_rtk_topic')),lambda m,who=who:self._on_fix(m,who),10)
        self.timer=self.create_timer(1./float(value('publish_hz')),self._tick)

    def _now(self):return self.get_clock().now().nanoseconds*1e-9

    def _on_fix(self,m,who):
        now=self._now();watch=self.watch[who]
        if self.rtk_type=='navsatfix':
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            xyz=(m.latitude,m.longitude,m.altitude)
            cov=np.asarray(m.position_covariance,float)
            good=(m.status.status>=0 and m.position_covariance_type!=0 and len(cov)==9
                  and np.all(np.isfinite(cov)) and np.all(cov[[0,4,8]]>0)
                  and float(np.sqrt(max(cov[0],cov[4],cov[8])))<=self.sigma_max)
            if good:
                matrix=cov.reshape(3,3)
                good=bool(np.allclose(matrix,matrix.T) and np.min(np.linalg.eigvalsh(matrix))>=0)
                sigma=float(np.sqrt(np.max(np.linalg.eigvalsh(matrix))))
        else:
            # array contract: [source_stamp, x/lat, y/lon, z/alt, sigma_m].
            if len(m.data)<5:watch.valid=False;return
            stamp,*xyz,sigma=list(m.data[:5])
            good=math.isfinite(sigma) and 0<sigma<=self.sigma_max
        if not good:
            watch.valid=False;return
        if not watch.accept(stamp,xyz,now,time.monotonic()):return
        if self.use_geo and not (-90<=xyz[0]<=90 and -180<=xyz[1]<=180):
            watch.valid=False;return
        if self.origin is None and self.use_geo:self.origin=tuple(xyz)
        self.fix[who]=(stamp,np.asarray(xyz,float))
        self.sigma[who]=sigma

    def _px4_fresh(self,m,key,values):
        if m.timestamp<=self.px4_stamp.get(key,0):return False
        self.px4_stamp[key]=m.timestamp
        return self.watch[key].accept(self._now(),values,self._now(),time.monotonic())

    def _on_px4(self,m,who):
        key=who+'_pos';values=(m.x,m.y,m.z,m.vx,m.vy,m.vz)
        if not (m.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid):
            self.watch[key].valid=False;return
        if self._px4_fresh(m,key,values):
            self.pose[who]=np.asarray(values[:3],float)+(self.a_off if who=='a' else self.b_off)

    def _on_att(self,m,who):
        key=who+'_att'
        if not (np.all(np.isfinite(m.q)) and .8<=float(np.dot(m.q,m.q))<=1.2):
            self.watch[key].valid=False;return
        if self._px4_fresh(m,key,m.q):self.att[who]=quat_to_rpy(m.q)

    def _tick(self):
        now=self._now();mono=time.monotonic()
        required=('a','b','b_pos','a_att','b_att') if self.source=='rtk' else ('a_pos','b_pos')
        good=all(self.watch[k].fresh(now,mono) for k in required)
        if self.source=='rtk' and good:
            good=abs(self.fix['a'][0]-self.fix['b'][0])<=self.sync
            good=good and math.hypot(self.sigma['a'],self.sigma['b'])<=self.sigma_max
        self.valid_pub.publish(Bool(data=good))
        if not good:return
        source=min(self.watch[k].stamp for k in required)
        if self.last_source is not None and source<=self.last_source:return
        if self.source=='rtk':
            a,b=(self.fix[k][1] for k in ('a','b'))
            if self.use_geo:
                a=geodetic_to_ned(*a,*self.origin);b=geodetic_to_ned(*b,*self.origin)
            # Publish A vehicle origin in B world frame, so B's geometry isn't shifted twice.
            # Compare antenna references, compensate A antenna->vehicle; B remains antenna
            # plus its measured antenna->vehicle correction in the common NED frame.
            a=compensate_lever_arm(a,self.a_lever,*self.att['a'])
            b=compensate_lever_arm(b,self.b_lever,*self.att['b'])
            p=a_state_for_b(a,b,self.pose['b'])
        else:p=self.pose['a']
        v=self.vel.update(source,p)
        self.last_source=source
        self.pub.publish(Float64MultiArray(data=[float(source),*p.tolist(),*v.tolist()]))


def main(args=None):
    rclpy.init(args=args);node=RelNavNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
