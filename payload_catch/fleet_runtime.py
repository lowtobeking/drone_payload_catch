"""ROS-free fleet configuration and bidirectional link lease."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROLES = {'a': ('safety_supervisor', 'release_guard', 'payload_servo', 'a_node'),
         'b': ('relnav_node', 'b_node')}


def load_fleet(path):
    import yaml
    path = Path(path).resolve()
    d = yaml.safe_load(path.read_text())
    if d.get('schema') != 1 or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', d.get('mission_id', '')) or d['mission_id']=='FILL_ME':
        raise ValueError('fill a unique mission_id and schema=1')
    if type(d.get('domain_id')) is not int or not 0 <= d['domain_id'] <= 100:
        raise ValueError('domain_id must be 0..100')
    if d['domain_id']==0 and d.get('domain_zero_confirmed') is not True:
        raise ValueError('domain 0 requires explicit measured FC/Agent domain confirmation')
    ips = []
    for role in ('a', 'b'):
        host = d['hosts'][role]
        ip = ipaddress.IPv4Address(host['ip'])
        if ip.is_loopback or ip.is_multicast or ip.is_unspecified or ip.is_link_local:
            raise ValueError('fill actual LAN addresses')
        ips.append(str(ip))
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', host['hostname']):
            raise ValueError('fill actual hostnames')
        if host['system_id'] != (1 if role == 'a' else 2):
            raise ValueError('PX4 system IDs must be A=1, B=2')
    if len(set(ips)) != 2 or d['hosts']['a']['hostname'] == d['hosts']['b']['hostname']:
        raise ValueError('A/B must be distinct hosts and IPs')
    for ip in d.get('operator_ips', []):
        ipaddress.IPv4Address(ip)
    mission = d.get('launch', {})
    if d.get('geometry_confirmed') is not True:
        raise ValueError('measure geometry and set geometry_confirmed=true')
    for key in ('a_lever', 'b_lever', 'b_offset', 'a_hover', 'b_standby'):
        v = mission[key]
        if not isinstance(v, list) or len(v)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in v):
            raise ValueError('fill measured vector: '+key)
    if mission.get('coord_mode', 'handshake') != 'handshake':
        raise ValueError('distributed flight requires handshake')
    if mission.get('preflight_require_gps') not in ('true', 'false'):
        raise ValueError('select GPS or explicitly validated indoor positioning')
    servo = Path(d['servo_config'])
    if not servo.is_absolute(): servo = path.parent / servo
    servo = servo.resolve()
    raw = yaml.safe_load(servo.read_text())['payload_servo']['ros__parameters']
    from .payload_servo import ServoConfig
    ServoConfig(**{k:v for k,v in raw.items() if k in ServoConfig.__dataclass_fields__}).validate()
    if raw.get('backend') != 'jetson':
        raise ValueError('fleet config requires jetson backend')
    digest = hashlib.sha256(json.dumps(dict(fleet=d,servo=raw),sort_keys=True,allow_nan=False).encode()).hexdigest()
    return d, servo, digest


def dds_profile(local_ip, peers):
    ns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'
    ET.register_namespace('',ns)
    def node(parent,name,text=None,**attrs):
        e=ET.SubElement(parent,'{'+ns+'}'+name,attrs)
        if text is not None:e.text=str(text)
        return e
    root=ET.Element('{'+ns+'}profiles')
    transport=node(node(root,'transport_descriptors'),'transport_descriptor')
    node(transport,'transport_id','fleet_udp');node(transport,'type','UDPv4')
    node(transport,'maxInitialPeersRange',64)
    node(node(transport,'interfaceWhiteList'),'address',str(ipaddress.IPv4Address(local_ip)))
    rtps=node(node(root,'participant',profile_name='fleet',is_default_profile='true'),'rtps')
    initial=node(node(rtps,'builtin'),'initialPeersList')
    for ip in sorted(set(peers)):
        udp=node(node(initial,'locator'),'udpv4')
        node(udp,'address',str(ipaddress.IPv4Address(ip)));node(udp,'port',0)
    node(node(rtps,'userTransports'),'transport_id','fleet_udp')
    node(rtps,'useBuiltinTransports','false')
    return ET.tostring(root,encoding='unicode')


class PeerLease:
    def __init__(self,role,fingerprint,boot):
        if role not in ROLES or not fingerprint or not boot:raise ValueError('role / fingerprint / boot required')
        self.role,self.peer=role,('b' if role=='a' else 'a')
        self.fingerprint,self.boot=fingerprint,boot
        self.peer_boot=None;self.stamp=self.received=self.remote=None;self.seq=-1;self.fault=''

    def accept(self,d,wall,mono):
        try:
            if d['role']!=self.peer or d['fingerprint']!=self.fingerprint:
                self.fault='peer role / release / configuration mismatch';return False
            if not isinstance(d['boot'],str) or not d['boot']:raise ValueError('peer boot missing')
            if self.peer_boot is not None and self.peer_boot!=d['boot']:
                self.fault='peer monitor restarted; ground restart required';return False
            if (type(d['seq']) is not int or d['seq']<=self.seq or not math.isfinite(d['stamp'])
                    or abs(wall-d['stamp'])>.02):return False
            if self.stamp is not None and d['stamp']<=self.stamp:return False
            self.peer_boot,self.stamp,self.seq=d['boot'],d['stamp'],d['seq']
            self.received,self.remote=mono,d
            return True
        except (KeyError,TypeError,ValueError):
            self.fault='invalid peer heartbeat';return False

    def ready(self,wall,mono,local_ok):
        d=self.remote or {}
        age=d.get('peer_rx_age_s')
        acknowledged=(type(age) in (int,float) and math.isfinite(age) and 0<=age<=.3)
        return (acknowledged and not self.fault and local_ok and self.received is not None
                and 0<=mono-self.received<=.3 and abs(wall-self.stamp)<=.32
                and d.get('local_ok') is True and d.get('seen_peer_boot')==self.boot)
