"""Read A's complete live PX4 parameter list from its existing eno1 MAVLink link."""
import json
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, "/home/nvidia/.local/lib/python3.10/site-packages")
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.a_solo_preflight import decode_parameter


def main():
    if Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode() != "1612425621719":
        raise RuntimeError("not verified A board")
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True
    with socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(0x0800)) as sniff:
        sniff.bind(("eno1", 0))
        sniff.settimeout(0.3)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind(("10.41.10.100", 0))
            class Writer:
                def write(self, data):
                    sender.sendto(data, ("10.41.10.2", 14550))
            tx = mavutil.mavlink.MAVLink(Writer(), srcSystem=244, srcComponent=191)
            def receive():
                try:
                    packet = sniff.recv(65535)
                except socket.timeout:
                    return []
                if len(packet) < 28 or packet[0] >> 4 != 4 or packet[9] != 17:
                    return []
                head = (packet[0] & 15) * 4
                if len(packet) < head + 8 or socket.inet_ntoa(packet[12:16]) != "10.41.10.2":
                    return []
                port, _, length, _ = struct.unpack("!HHHH", packet[head:head+8])
                if port != 14550 or length < 8 or len(packet) < head + length:
                    return []
                return [m for m in (parser.parse_buffer(packet[head+8:head+length]) or [])
                        if (m.get_srcSystem(), m.get_srcComponent()) == (1, 1)]
            heartbeat = None
            until = time.monotonic() + 6
            while time.monotonic() < until and heartbeat is None:
                for msg in receive():
                    if msg.get_type() == "HEARTBEAT":
                        heartbeat = msg
            if heartbeat is None or heartbeat.autopilot != 12:
                raise RuntimeError("A PX4 heartbeat missing on eno1")
            tx.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
            version = None
            until = time.monotonic() + 5
            while time.monotonic() < until and version is None:
                for msg in receive():
                    if msg.get_type() == "AUTOPILOT_VERSION":
                        version = msg
            if version is None or version.uid != 3544984969635313204:
                raise RuntimeError("A PX4 identity not verified")
            bytewise = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
            ccast = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
            if bytewise == ccast:
                raise RuntimeError("ambiguous PX4 parameter encoding")
            tx.param_request_list_send(1, 1)
            params, indices, count = {}, {}, None
            until = time.monotonic() + 45
            while time.monotonic() < until:
                for msg in receive():
                    if msg.get_type() != "PARAM_VALUE":
                        continue
                    if count is not None and msg.param_count != count:
                        raise RuntimeError("PX4 parameter count changed")
                    count = msg.param_count
                    key = msg.param_id.decode("ascii") if isinstance(msg.param_id, bytes) else msg.param_id
                    key = key.rstrip("\0")
                    if key == "_HASH_CHECK":
                        continue
                    if not 0 <= msg.param_index < count:
                        raise RuntimeError("PX4 parameter index invalid")
                    val = decode_parameter(msg.param_value, msg.param_type, bytewise)
                    if (msg.param_index in indices and indices[msg.param_index] != key or
                            key in params and params[key] != val):
                        raise RuntimeError("PX4 parameter snapshot changed")
                    indices[msg.param_index], params[key] = key, val
                if count and len(indices) == count:
                    break
            if not count or set(indices) != set(range(count)) or len(params) != count:
                raise RuntimeError("PX4 parameter list incomplete")
            print(json.dumps(params, allow_nan=False))


if __name__ == "__main__":
    main()
