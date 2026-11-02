#!/usr/bin/python3
"""Maintain one Wi-Fi route for the authorized laptop, learned from ARP.

Never modifies NetworkManager profiles, default routes, FC routes, DNS or sysctls.
Only owned proto-186 /32 routes with metric 9 may be removed by this service.
"""
import ipaddress
import json
import logging
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time

IFACE = "wlP1p1s0"
LAPTOP_MAC = bytes.fromhex("985f416480ed")
JETSON_MAC = bytes.fromhex("dc4a9e38207f")
PROFILE = "5bf25dec-358c-4a85-9eed-ee5b19a254ca"
PROTO = 186
METRIC = 9
STATE = Path("/run/payload-catch-wifi-peer.json")
# Reserved static endpoints must never be taken over by this laptop rule.
EXCLUDED = {"192.168.1.1", "192.168.1.101", "192.168.1.111",
            "192.168.1.123", "192.168.1.200", "192.168.1.202", "192.168.1.203"}
STOP = False

def command(args):
    p = subprocess.run(args, capture_output=True, text=True, timeout=5, shell=False)
    if p.returncode:
        raise RuntimeError(f"{args!r}: exit={p.returncode} stderr={p.stderr}")
    return p.stdout

def valid_peer(ip, local):
    address = ipaddress.IPv4Address(ip)
    net = ipaddress.IPv4Interface(local).network
    return (address in net and address not in (net.network_address, net.broadcast_address)
            and ip != str(ipaddress.IPv4Interface(local).ip) and ip not in EXCLUDED)

def parse_arp(frame):
    if len(frame) < 42 or frame[12:14] != b"\x08\x06":
        return None
    hardware, protocol, hlen, plen, operation = struct.unpack("!HHBBH", frame[14:22])
    if (hardware, protocol, hlen, plen) != (1, 0x800, 6, 4) or operation not in (1, 2):
        return None
    if frame[6:12] != frame[22:28]:
        return None
    return frame[22:28], socket.inet_ntoa(frame[28:32])

def route_rows():
    return json.loads(command(["ip", "-j", "-4", "route", "show", "table", "main"]))

def is_owned(row, peer=None):
    return (row.get("dev") == IFACE and str(row.get("protocol")) in {str(PROTO), "bgp"}
            and row.get("metric") == METRIC
            and (peer is None or row.get("dst") in (peer, peer + "/32")))

class RouteManager:
    def __init__(self):
        self.peer = None
        self.last_seen = 0.

    def clear(self):
        # Protocol, device, prefix and metric all constrain deletion.
        for row in route_rows():
            if is_owned(row) and ipaddress.ip_network(row["dst"], strict=False).prefixlen == 32:
                command(["ip", "-4", "route", "del", row["dst"], "dev", IFACE,
                         "metric", str(METRIC), "proto", str(PROTO)])
                logging.info("Removed own route %s", row["dst"])
        self.peer = None
        self.last_seen = 0.
        STATE.unlink(missing_ok=True)

    def observe(self, mac, ip, local, now):
        if self.peer == ip and mac != LAPTOP_MAC:
            logging.warning("Peer IP claimed by another MAC; remove own route")
            self.clear()
            return False
        if mac != LAPTOP_MAC or not valid_peer(ip, local):
            return False
        rows = route_rows()
        existing = [r for r in rows if r.get("dst") in (ip, ip + "/32")]
        if any(r.get("dev") != IFACE for r in existing):
            logging.warning("Existing host route on another device; refusing %s", ip)
            return False
        if self.peer != ip:
            self.clear()
        if not any(is_owned(r, ip) for r in existing):
            # Add a separately owned route; do not replace existing configuration.
            command(["ip", "-4", "route", "add", ip + "/32", "dev", IFACE,
                     "scope", "link", "metric", str(METRIC), "proto", str(PROTO)])
            logging.info("Authorized laptop %s -> Wi-Fi", ip)
        self.peer, self.last_seen = ip, now
        STATE.write_text(json.dumps({"peer": ip, "mac": LAPTOP_MAC.hex(":"),
                                     "interface": IFACE, "last_seen_monotonic": now})+"\n",
                         encoding="utf-8")
        return True

    def expire(self, now):
        if self.peer and now - self.last_seen > 120:
            logging.info("ARP identity lease expired")
            self.clear()

def context():
    active = command(["nmcli", "-g", "GENERAL.CON-UUID", "device", "show", IFACE]).strip()
    if active != PROFILE:
        return None
    rows = json.loads(command(["ip", "-j", "address", "show", "dev", IFACE]))
    for row in rows:
        if row.get("address") != JETSON_MAC.hex(":"):
            raise RuntimeError("Unexpected Wi-Fi adapter MAC")
        for a in row.get("addr_info", []):
            if a.get("scope") == "global" and a.get("family") == "inet":
                return a["local"] + "/" + str(a["prefixlen"])
    return None

def request(sock, local, target):
    own = socket.inet_aton(str(ipaddress.IPv4Interface(local).ip))
    packet = (b"\xff"*6 + JETSON_MAC + b"\x08\x06"
              + struct.pack("!HHBBH", 1, 0x800, 6, 4, 1)
              + JETSON_MAC + own + bytes(6) + socket.inet_aton(target))
    sock.send(packet)

def stop(signum, frame):
    global STOP
    STOP = True

def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    serial = Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
    if serial != "1612425621719":
        raise RuntimeError("Wrong Jetson")
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)
    manager = RouteManager()
    manager.clear()
    try:
        with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as sock:
            sock.bind((IFACE, 0))
            local = None
            next_context, next_request = 0., 0.
            while not STOP:
                now = time.monotonic()
                if now >= next_context:
                    latest = context()
                    if latest != local:
                        manager.clear()
                        local = latest
                    next_context = now + 5
                if local and now >= next_request:
                    target = manager.peer or "192.168.1.4"
                    if valid_peer(target, local):
                        request(sock, local, target)
                    next_request = now + 15
                if select.select([sock], [], [], 1)[0]:
                    frame = sock.recv(2048)
                    decoded = parse_arp(frame)
                    if decoded and local:
                        manager.observe(*decoded, local, time.monotonic())
                manager.expire(time.monotonic())
    finally:
        manager.clear()

if __name__ == "__main__":
    main()
