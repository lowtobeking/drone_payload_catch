#!/usr/bin/env python3
"""Project-owned FC/QGC bridge. Only the configured GCS may send toward the FC.

Telemetry broadcast deduplication follows the IP_PKTINFO strategy previously
validated on this Jetson. No automatic learning from arbitrary network hosts.
This forwards MAVLink bytes; the FC remains responsible for protocol validation.
"""
import argparse
import errno
import ipaddress
import signal
import select
import socket
import struct
import time


class Routes:
    def __init__(self,fc_ip,gcs_ip,port):
        self.fc=(fc_ip,port);self.gcs=(gcs_ip,port)
        self.last_unicast=None

    def destination(self,data,source,destination_ip,now):
        if not data or data[0] not in (0xfd,0xfe):return None
        if source[0]==self.fc[0]:
            broadcast=destination_ip is not None and destination_ip.endswith('.255')
            if broadcast and self.last_unicast is not None and now-self.last_unicast<=3.:
                return None
            if not broadcast:self.last_unicast=now
            return self.gcs
        if source[0]==self.gcs[0]:
            self.gcs=source
            return self.fc
        return None


def destination_ip(ancillary):
    for level,kind,data in ancillary:
        if level==socket.IPPROTO_IP and kind==8 and len(data)>=12:
            return socket.inet_ntoa(struct.unpack('I4s4s',data[:12])[2])
    return None


def forward(stream,data,target):
    # UDP queues may fill while the configured GCS is offline. Drop this packet;
    # never block the receive loop or replay a delayed control packet later.
    try:
        return stream.sendto(data,target)==len(data)
    except OSError:
        return False


def receive(stream):
    try:
        return stream.recvmsg(65535,socket.CMSG_SPACE(64))
    except BlockingIOError:
        return None
    except OSError as exc:
        if exc.errno in (errno.EHOSTUNREACH,errno.ENETUNREACH,errno.ECONNREFUSED,errno.ENOBUFS):
            return None
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fc-ip',required=True);p.add_argument('--gcs-ip',required=True)
    p.add_argument('--port',type=int,default=14550)
    a=p.parse_args()
    for ip in (a.fc_ip,a.gcs_ip):
        value=ipaddress.IPv4Address(ip)
        if value.is_unspecified or value.is_multicast:p.error('specific FC/GCS addresses required')
    if a.fc_ip==a.gcs_ip or not 1<=a.port<=65535:p.error('distinct FC/GCS and valid port required')
    stop=False
    def stopping(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
    route=Routes(a.fc_ip,a.gcs_ip,a.port);counts={'fc_to_gcs':0,'gcs_to_fc':0,'dropped':0,'send_dropped':0}
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
        # No SO_REUSEADDR: another controller must not silently share this port.
        stream.setsockopt(socket.IPPROTO_IP,8,1)
        stream.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        stream.setblocking(False);stream.bind(('0.0.0.0',a.port))
        print('payload_catch relay: FC='+a.fc_ip+' GCS='+a.gcs_ip+' port='+str(a.port),flush=True)
        report_at=time.monotonic()+30
        while not stop:
            readable,_,_=select.select([stream],[],[],.5)
            if not readable:continue
            packet=receive(stream)
            if packet is None:continue
            data,ancillary,_,source=packet
            target=route.destination(data,source,destination_ip(ancillary),time.monotonic())
            if target is None:counts['dropped']+=1
            else:
                if forward(stream,data,target):counts['gcs_to_fc' if target==route.fc else 'fc_to_gcs']+=1
                else:counts['send_dropped']+=1
            if time.monotonic()>=report_at:
                print(str(counts),flush=True);report_at=time.monotonic()+30
    print('relay stopped '+str(counts),flush=True)


if __name__=='__main__':main()
