#!/usr/bin/env python3
"""Independent last release interlock. Hardware adapters consume authorized_release only."""
import json
import time
import uuid
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from .safety_runtime import SourceWatch


class ReleaseGuard(Node):
    def __init__(self):
        super().__init__('release_guard')
        self.declare_parameter('authority',0)
        self.owner=int(self.get_parameter('authority').value)
        self.declare_parameter('release_delay_s',0.0)
        self.release_delay_s=float(self.get_parameter('release_delay_s').value)
        if not math.isfinite(self.release_delay_s) or not 0<=self.release_delay_s<=3.0:
            raise ValueError('invalid mechanical release delay')
        if self.owner not in (0,1):raise ValueError('release authority must be 0 or 1')
        self.permit_watch=SourceWatch(.2);self.supervisor_watch=SourceWatch()
        self.allow=False;self.supervisor=None;self.release_at=None;self.released=False
        self.last_schedule=-math.inf
        self.aborted=False
        self.pub=self.create_publisher(String,'/payload/authorized_release',10)
        self.create_subscription(String,'/safety/state',self._supervisor,10)
        self.create_subscription(String,'/payload/release_permit',self._permit,10)
        self.create_subscription(Float64,'/payload/release_at',self._schedule,10)
        self.create_subscription(Bool,'/payload/release_abort',self._abort,10)
        self.timer=self.create_timer(.01,self._tick)

    def _supervisor(self,m):
        try:
            d=json.loads(m.data)
            if not isinstance(d['session'],str) or not d['session']:
                raise ValueError('missing safety session')
            if self.supervisor_watch.accept(d['stamp'],[],time.time(),time.monotonic()):
                if self.supervisor is not None and self.supervisor['session']!=d['session']:
                    self._abort(Bool(data=True))
                self.supervisor=d
        except (ValueError,KeyError,TypeError):self.supervisor_watch.valid=False

    def _permit(self,m):
        try:
            d=json.loads(m.data)
            if d['owner']!=self.owner:return
            if self.permit_watch.accept(d['stamp'],[],time.time(),time.monotonic()):self.allow=d['allow'] is True
        except (ValueError,KeyError,TypeError):self.permit_watch.valid=False

    def _schedule(self,m):
        now=self.get_clock().now().nanoseconds*1e-9
        if (not self.released and not self.aborted and self.release_at is None and math.isfinite(m.data) and 0<=m.data-now<=5
                and (self.release_delay_s==0 or m.data-self.release_delay_s>now)
                and m.data>self.last_schedule):
            self.release_at=float(m.data)
            self.last_schedule=self.release_at

    def _abort(self,m):
        if m.data:
            self.aborted=True
            self.release_at=None;self.allow=False;self.permit_watch.valid=False

    def _tick(self):
        if self.released or self.release_at is None:return
        now=self.get_clock().now().nanoseconds*1e-9;wall=time.time();mono=time.monotonic()
        planned_at=self.release_at
        command_at=planned_at-self.release_delay_s
        if now<command_at:return
        valid=(now-command_at<=(.03 if self.release_delay_s else .10) and self.allow and
               self.permit_watch.fresh(wall,mono) and self.supervisor_watch.fresh(wall,mono)
               and self.supervisor is not None and
               self.supervisor.get('release_permitted') is True and
               all(self.supervisor.get('permits',{}).get(str(i)) is True for i in (0,1))
               and all(self.supervisor.get('actions',{}).get(str(i))=='run' for i in (0,1)))
        self.release_at=None
        if not valid:
            self._abort(Bool(data=True))
            self.get_logger().error('RELEASE BLOCKED: expired permit / safety / late schedule');return
        self.released=True
        self.pub.publish(String(data=json.dumps(dict(stamp=wall,nonce=uuid.uuid4().hex,owner=self.owner,
            session=self.supervisor['session'],planned_at=planned_at,command_at=command_at))))


def main(args=None):
    rclpy.init(args=args);node=ReleaseGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
