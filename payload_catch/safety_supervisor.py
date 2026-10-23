#!/usr/bin/env python3
"""Independent operator safety and PX4 command acknowledgement supervisor."""
from __future__ import annotations
import json
import time
import uuid
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from px4_msgs.msg import (VehicleCommand, VehicleCommandAck, VehicleStatus,
    VehicleLocalPosition, VehicleAttitude, BatteryStatus, FailsafeFlags,
    EstimatorStatusFlags, SensorGps, ManualControlSetpoint, VehicleLandDetected)
from .px4_iface import qos_in, qos_out, topic_for
from .preflight_health import HealthAccumulator
from .safety_runtime import (LEASE_S, FAULT_LAND_S, flight_reasons,
    CommandTicket, SourceWatch, landing_order)


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('require_rc', True)
        self.require_rc = bool(self.get_parameter('require_rc').value)
        self.health = {i:HealthAccumulator() for i in (0,1)}
        self.status = {}; self.position = {}; self.landed = {}
        self.task = {}; self.task_watch = {i:SourceWatch() for i in (0,1)}
        self.action = {i:'run' for i in (0,1)}
        self.reason = {i:'' for i in (0,1)}
        self.resume_token = {i:'' for i in (0,1)}
        self.resume_until = {}
        self.fault_since = {}; self.tickets = {}; self.ack_stamp = {}
        self.land_targets = set(); self.released = False
        self.session = uuid.uuid4().hex; self.seq = 0
        self.command_pub = {i:self.create_publisher(VehicleCommand,
            topic_for(i,'in/vehicle_command'),qos_out()) for i in (0,1)}
        self.state_pub = self.create_publisher(String,'/safety/state',10)
        self.abort_pub = self.create_publisher(Bool,'/payload/release_abort',10)
        for action in ('pause','land','kill','resume'):
            self.create_subscription(Bool,'/safety/'+action,
                lambda m,action=action:self._operator(action,(0,1)) if m.data else None,10)
            for i,label in ((0,'a'),(1,'b')):
                self.create_subscription(Bool,f'/safety/{action}_{label}',
                    lambda m,i=i,action=action:self._operator(action,(i,)) if m.data else None,10)
        self.create_subscription(String,'/safety/request',self._request,10)
        self.create_subscription(Bool,'/payload/released',self._released,10)
        for i,label in ((0,'a'),(1,'b')):
            self.create_subscription(String,'/safety/task_'+label,lambda m,i=i:self._task(m,i),10)
            self.create_subscription(VehicleCommandAck,topic_for(i,'out/vehicle_command_ack'),
                                     lambda m,i=i:self._ack(m,i),qos_in())
            self.create_subscription(VehicleLandDetected,topic_for(i,'out/vehicle_land_detected'),
                                     lambda m,i=i:self._landed(m,i),qos_in())
            for cls,key,suffix in ((VehicleStatus,'status','vehicle_status_v1'),
                (VehicleLocalPosition,'lpos','vehicle_local_position'),
                (VehicleAttitude,'att','vehicle_attitude'),(BatteryStatus,'batt','battery_status'),
                (FailsafeFlags,'failsafe','failsafe_flags'),(EstimatorStatusFlags,'est','estimator_status_flags'),
                (SensorGps,'gps','vehicle_gps_position'),(ManualControlSetpoint,'rc','manual_control_setpoint')):
                self.create_subscription(cls,topic_for(i,'out/'+suffix),
                                         lambda m,i=i,key=key:self._observe(m,i,key),qos_in())
        self.timer = self.create_timer(.05,self._tick)

    def _observe(self,m,i,key):
        now=time.monotonic(); acc=self.health[i]
        before=acc.last_stamp.get(key,0); acc.record(key,m,now)
        if m.timestamp <= before: return
        if key=='status': self.status[i]=(now,m)
        if key=='lpos' and m.xy_valid and m.z_valid and all(math.isfinite(x) for x in (m.x,m.y,m.z)):
            self.position[i]=(now,[m.x,m.y,m.z])

    def _landed(self,m,i):
        if m.timestamp > self.ack_stamp.get(('land',i),0):
            self.ack_stamp['land',i]=m.timestamp
            self.landed[i]=(time.monotonic(),bool(m.landed))

    def _released(self,m):
        if m.data: self.released=True

    def _task(self,m,i):
        try:
            d=json.loads(m.data)
            if self.task_watch[i].accept(d['stamp'],[],time.time(),time.monotonic()):
                self.task[i]=d
        except (ValueError,KeyError,TypeError):
            self.task_watch[i].valid=False

    def _request(self,m):
        try:
            d=json.loads(m.data)
            i=int(d['drone_id'])
            if i not in (0,1) or not 0 <= time.time()-d['stamp'] <= LEASE_S: return
            self._operator(d['action'],(i,),str(d.get('reason','task request')))
        except (ValueError,KeyError,TypeError): pass

    def _operator(self,action,ids,reason='operator'):
        now=time.monotonic()
        if action not in ('pause','land','kill','resume'): return
        if action=='resume':
            ids=(0,1) # handover resumes as a pair, including single-aircraft input
            if self.released or self.land_targets or any(self.action[i] in ('kill','manual') for i in ids):
                self.get_logger().error('RESUME DENIED: released/landing/RC takeover; restart requires preflight')
                return
            if any(flight_reasons(self.health[i].snapshot(now),require_rc=self.require_rc)
                   or not self.task_watch[i].fresh(time.time(),now)
                   or self.task.get(i,{}).get('sources_ready') is not True
                   or self.task.get(i,{}).get('released') is True for i in (0,1)):
                self.get_logger().error('RESUME DENIED: dual-aircraft health/tasks unavailable'); return
            for i in ids:
                self.action[i]='run'; self.reason[i]='operator resume'
                self.resume_token[i]=uuid.uuid4().hex; self.tickets.pop(i,None)
                self.fault_since.pop(i,None)
                self.resume_until[i]=now+2.
            return
        self.abort_pub.publish(Bool(data=True))
        # Stopping one member cancels release authority for the entire pair.
        for i in (0,1):
            if self.action[i]=='run': self.action[i]='hold'; self.reason[i]=reason
        for i in ids:
            if self.action[i]=='kill': continue
            if action=='kill': self.action[i]='kill'; self.reason[i]=reason; self.land_targets.discard(i)
            elif action=='land':
                self.land_targets.add(i); self.reason[i]=reason
                if reason=='operator': self.action[i]='land'
            elif self.action[i] not in ('land','manual'):
                self.action[i]='hold'; self.reason[i]=reason

    def _ack(self,m,i):
        if m.timestamp <= self.ack_stamp.get(i,0): return
        self.ack_stamp[i]=m.timestamp
        ticket=self.tickets.get(i)
        if ticket: ticket.acknowledge(m.command,m.result,m.target_system,m.target_component)

    def _send(self,i,ticket):
        m=VehicleCommand();m.command=ticket.command
        if ticket.action=='kill': m.param1=1.
        elif ticket.action=='hold': m.param1=1.;m.param2=4.;m.param3=3. # PX4 AUTO LOITER
        m.target_system=i+1;m.target_component=1;m.source_system=255;m.source_component=191
        m.from_external=True;m.timestamp=int(time.time()*1e6)
        self.command_pub[i].publish(m);ticket.sent(time.monotonic())

    def _tick(self):
        now=time.monotonic(); wall=time.time()
        snaps={i:a.snapshot(now) for i,a in self.health.items()}
        status_fresh={i:i in self.status and now-self.status[i][0] <= 2 for i in (0,1)}
        armed={i:i in self.status and self.status[i][1].arming_state==2 for i in (0,1)}
        landed={i:i in self.landed and now-self.landed[i][0] <= 2 and self.landed[i][1]
                and status_fresh[i] and not armed[i] for i in (0,1)}
        healthy={i:not flight_reasons(snaps[i],require_rc=self.require_rc) for i in (0,1)}
        tasks={i:self.task_watch[i].fresh(wall,now) for i in (0,1)}
        for i in (0,1):
            nav=self.status[i][1].nav_state if status_fresh[i] else None
            ever=self.task.get(i,{}).get('ever_offboard') is True
            task_safe=(self.task.get(i,{}).get('safety')=='OK' or
                       (now<self.resume_until.get(i,0) and self.task.get(i,{}).get('safety')=='HOLD'))
            fault=(not healthy[i] or not tasks[i]
                   or self.task.get(i,{}).get('sources_ready') is not True
                   or (not task_safe and self.reason[i]!='operator'))
            # Independently track faults after task-initiated/operator HOLD too.
            if armed[i] and ever and self.action[i] not in ('manual','kill') and fault:
                self.fault_since.setdefault(i,now)
            elif not fault:
                self.fault_since.pop(i,None)
            if armed[i] and self.task.get(i,{}).get('ever_offboard') and self.action[i]=='run':
                if status_fresh[i] and nav!=14 and now>=self.resume_until.get(i,0):
                    self._operator('pause',(0,1),'RC / PX4 mode takeover')
                    self.action[i]='manual';self.tickets.pop(i,None)
                elif (not healthy[i] or not tasks[i] or not task_safe
                      or self.task.get(i,{}).get('sources_ready') is not True):
                    self._operator('pause',(0,1),'flight health / task fault')
                    self.fault_since.setdefault(i,now)
            ticket=self.tickets.get(i)
            if (status_fresh[i] and armed[i] and self.action[i] in ('hold','land')
                    and ((ticket is not None and ticket.mode_confirmed and nav!=ticket.expected_mode)
                     or nav in (0,1,2,3,5,10,13,15)
                     or (self.action[i]=='hold' and nav==18))):
                self.action[i]='manual';self.land_targets.discard(i);self.tickets.pop(i,None)
                self.reason[i]='RC / PX4 takeover; safety mode retries stopped'
            if i in self.fault_since and now-self.fault_since[i]>=FAULT_LAND_S:
                # Never switch a pilot-controlled mode automatically.
                if self.action[i] not in ('manual','kill'): self.land_targets.update((0,1))
        positions={i:self.position[i][1] for i in self.position if now-self.position[i][0] <= 1}
        if self.land_targets:
            for i,action in landing_order(self.land_targets,positions,armed,landed).items():
                if self.action[i] not in ('kill','manual'): self.action[i]=action
        progress={}
        for i in (0,1):
            action=self.action[i]
            if action not in ('hold','land','kill') or (not armed[i] and action!='kill'):
                continue
            if action=='hold' and not healthy[i]:
                progress[i]='HOLD_UNAVAILABLE: relinquish to PX4 failsafe / RC'
                continue # no unsafe position/velocity command from supervisor
            command,mode=(185,13) if action=='kill' else ((21,18) if action=='land' else (176,4))
            ticket=self.tickets.get(i)
            if ticket is None or ticket.action!=action:
                ticket=CommandTicket(action,command,mode,now);self.tickets[i]=ticket
            if ticket.due(now): self._send(i,ticket)
            progress[i]=ticket.update(now,status_fresh=status_fresh[i],
                nav_state=self.status[i][1].nav_state if status_fresh[i] else -1,
                armed=armed[i],landed=landed[i])
        for i,ticket in self.tickets.items():
            if i not in progress:
                progress[i]=ticket.update(now,status_fresh=status_fresh[i],
                    nav_state=self.status[i][1].nav_state if status_fresh[i] else -1,
                    armed=armed[i],landed=landed[i])
        pair_ok=all(self.task.get(j,{}).get('safety')=='OK'
                    or (now<self.resume_until.get(j,0) and self.task.get(j,{}).get('safety')=='HOLD')
                    for j in (0,1))
        permits={i:self.action[i]=='run' and all(healthy.values()) and all(tasks.values())
                   and all(self.task.get(j,{}).get('sources_ready') is True for j in (0,1))
                   and pair_ok for i in (0,1)}
        self.seq+=1
        data=dict(session=self.session,seq=self.seq,stamp=wall,actions=self.action,
                  reasons=self.reason,permits=permits,resume_tokens=self.resume_token,
                  progress=progress,released=self.released,
                  release_permitted=all(permits.values()) and all(
                      status_fresh[i] and armed[i] and self.status[i][1].nav_state==14
                      for i in (0,1)))
        self.state_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args);node=SafetySupervisor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
