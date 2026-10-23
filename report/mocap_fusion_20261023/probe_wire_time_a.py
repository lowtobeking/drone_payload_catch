"""Passive VRPN time comparison. No FC, clock writes or server configuration."""
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
import json
import os
import selectors
import socket
import struct
import subprocess
import time

SERVER = "192.168.1.111"
READER = "/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0/planeA_stream"


def main():
    if Path("/proc/device-tree/serial-number").read_text().strip("\0\n") != "1612425621719":
        raise RuntimeError("wrong device")
    child = None
    selector = selectors.DefaultSelector()
    callback = {}
    wire = []
    buffer = b""
    with socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(0x0800)) as sniff:
        sniff.bind(("wlP1p1s0", 0))
        sniff.setblocking(False)
        selector.register(sniff, selectors.EVENT_READ, "network")
        try:
            child = subprocess.Popen(
                [READER, "192.168.1.4"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
            os.set_blocking(child.stdout.fileno(), False)
            selector.register(child.stdout, selectors.EVENT_READ, "reader")
            start = time.monotonic()
            while time.monotonic() - start < 8:
                for key, _ in selector.select(timeout=.005):
                    if key.data == "reader":
                        chunk = os.read(child.stdout.fileno(), 65536)
                        if not chunk:
                            raise RuntimeError("reader ended unexpectedly")
                        buffer += chunk
                        lines = buffer.split(b"\n")
                        buffer = lines.pop()
                        if len(buffer) > 262144:
                            raise RuntimeError("reader buffer overflow")
                        for line in lines:
                            row = json.loads(line)
                            stamp = row["server_stamp_us"]
                            callback[stamp] = row["tracker"]
                    else:
                        packet = sniff.recv(65535)
                        if len(packet) < 28 or packet[0] >> 4 != 4 or packet[9] != 17:
                            continue
                        if socket.inet_ntoa(packet[12:16]) != SERVER:
                            continue
                        ihl = (packet[0] & 15) * 4
                        if len(packet) < ihl + 8 or struct.unpack("!H", packet[6:8])[0] & 0x3fff:
                            continue
                        sport, dport, length, _ = struct.unpack("!HHHH", packet[ihl:ihl+8])
                        data = packet[ihl+8:ihl+length]
                        now = time.time_ns() // 1000
                        candidates = []
                        # Look for a timestamp pair in the UDP message/header.
                        # Exact matches against callbacks are evidence; plausible
                        # integers alone are NOT treated as decoded timestamps.
                        for offset in range(0, len(data)-7, 4):
                            sec, usec = struct.unpack("!II", data[offset:offset+8])
                            if abs(sec - now//1000000) < 366*86400 and usec < 1000000:
                                candidates.append({"offset": offset, "stamp_us": sec*1000000+usec})
                        if candidates:
                            wire.append({"local_receive_wall_us": now, "source_port": sport,
                                         "destination_port": dport, "payload_len": len(data),
                                         "header_hex": data[:32].hex(), "candidates": candidates})
        finally:
            selector.close()
            if child is not None:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=3)
                os.set_blocking(child.stdout.fileno(), True)
                remaining, stderr = child.communicate(timeout=3)
                for line in (buffer + remaining).splitlines():
                    if line:
                        row = json.loads(line)
                        callback[row["server_stamp_us"]] = row["tracker"]
    matches = []
    offsets = {}
    for row in wire:
        for candidate in row["candidates"]:
            if candidate["stamp_us"] in callback:
                offsets[candidate["offset"]] = offsets.get(candidate["offset"], 0) + 1
                matches.append({**candidate, "local_receive_wall_us": row["local_receive_wall_us"],
                                "source_minus_receive_us": candidate["stamp_us"]-row["local_receive_wall_us"],
                                "header_hex": row["header_hex"]})
    shanghai = ZoneInfo("Asia/Shanghai")
    def fmt(stamp):
        return datetime.fromtimestamp(stamp/1e6, shanghai).isoformat()
    example = matches[-1] if matches else None
    if example:
        example = {**example, "source_beijing": fmt(example["stamp_us"]),
                   "local_receive_beijing": fmt(example["local_receive_wall_us"])}
    result = {"read_only": True, "callback_frames": len(callback), "wire_packets": len(wire),
              "exact_wire_callback_matches": len(matches),
              "matched_timestamp_offsets_in_udp_payload": offsets,
              "example": example,
              "source_minus_receive_us_range": [min(m["source_minus_receive_us"] for m in matches),
                                                max(m["source_minus_receive_us"] for m in matches)] if matches else None,
              "reader_pid": child.pid, "reader_exit": child.returncode,
              "reader_stderr": stderr.decode("utf-8", errors="replace"),
              "clock_modified": False, "px4_modified": False,
              "conclusion": ("wire_contains_same_timestamp_as_callback" if len(matches) > 100
                             else "insufficient_evidence")}
    print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
