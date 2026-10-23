"""A FC read-only parameter backup using passive RX; leaves QGC's socket intact."""
import json
import socket
import struct
import sys
import time
from pathlib import Path
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, "/home/nvidia/payload_catch_deploy/current/src/tools")
from a_solo_preflight import decode_parameter


def main():
    assert Path("/proc/device-tree/serial-number").read_text().strip("\0\n") == "1612425621719"
    result = {"read_only": True, "complete": False, "transport": "passive eno1 RX plus read requests"}
    decoder = mavutil.mavlink.MAVLink(None)
    decoder.robust_parsing = True
    with socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(0x0800)) as sniff:
        sniff.bind(("eno1", 0))
        sniff.settimeout(.3)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind(("10.41.10.100", 0))
            class Writer:
                def __init__(self):
                    self.target = None
                def write(self, data):
                    if self.target is None:
                        raise RuntimeError("FC UDP port not observed")
                    sender.sendto(data, self.target)
            writer = Writer()
            tx = mavutil.mavlink.MAVLink(writer, srcSystem=244, srcComponent=191)
            def receive():
                try:
                    packet = sniff.recv(65535)
                except socket.timeout:
                    return []
                if len(packet) < 28 or packet[0] >> 4 != 4 or packet[9] != 17:
                    return []
                ihl = (packet[0] & 15) * 4
                if len(packet) < ihl + 8 or socket.inet_ntoa(packet[12:16]) != "10.41.10.2":
                    return []
                sport, _, length, _ = struct.unpack("!HHHH", packet[ihl:ihl+8])
                if sport != 14550 or length < 8 or len(packet) < ihl + length:
                    return []
                writer.target = ("10.41.10.2", sport)
                messages = decoder.parse_buffer(packet[ihl+8:ihl+length]) or []
                return [m for m in messages if (m.get_srcSystem(), m.get_srcComponent()) == (1, 1)]
            heartbeat = None
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                for msg in receive():
                    if msg.get_type() == "HEARTBEAT":
                        heartbeat = msg
                        break
                if heartbeat:
                    break
            if heartbeat is None or heartbeat.autopilot != 12:
                raise RuntimeError("PX4 A heartbeat not observed on eno1")
            result["heartbeat"] = heartbeat.to_dict()
            tx.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
            version = None
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                for msg in receive():
                    if msg.get_type() == "AUTOPILOT_VERSION":
                        version = msg
                if version:
                    break
            if version is None:
                raise RuntimeError("version response missing")
            result["version"] = version.to_dict()
            fw = int(version.flight_sw_version)
            result["firmware"] = [fw >> 24, (fw >> 16) & 255, (fw >> 8) & 255, fw & 255]
            bytewise = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
            ccast = bool(version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
            if bytewise == ccast:
                raise RuntimeError("ambiguous parameter encoding")
            result["encoding"] = "bytewise" if bytewise else "c_cast"
            tx.param_request_list_send(1, 1)
            params, indices, expected = {}, {}, None
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                for msg in receive():
                    if msg.get_type() != "PARAM_VALUE":
                        continue
                    key = msg.param_id.decode("ascii") if isinstance(msg.param_id, bytes) else msg.param_id
                    key = key.rstrip("\0")
                    if key == "_HASH_CHECK":
                        continue
                    if expected is not None and expected != msg.param_count:
                        raise RuntimeError("parameter count changed")
                    expected = msg.param_count
                    if not 0 <= msg.param_index < expected:
                        raise RuntimeError("invalid parameter index")
                    entry = {"value": decode_parameter(msg.param_value, msg.param_type, bytewise),
                             "type": msg.param_type, "index": msg.param_index}
                    if key in params and params[key] != entry:
                        raise RuntimeError("parameter changed during snapshot")
                    if msg.param_index in indices and indices[msg.param_index] != key:
                        raise RuntimeError("parameter index changed")
                    params[key], indices[msg.param_index] = entry, key
                if expected and set(indices) == set(range(expected)):
                    break
            result.update(parameters=params, expected=expected, received=len(indices),
                          complete=bool(expected and len(params) == expected
                                        and set(indices) == set(range(expected))))
    (ROOT / "parameters_backup_passive.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    summary = {k: v for k, v in result.items() if k != "parameters"}
    summary["ev_parameters"] = {k: v for k, v in result.get("parameters", {}).items()
                                if k.startswith(("EKF2_EV", "EKF2_HGT", "EKF2_MAG", "UXRCE_DDS"))}
    print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
