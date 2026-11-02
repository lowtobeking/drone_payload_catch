"""Transactional install of one laptop-specific route helper; no FC config changes."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

HELPER = "#!/usr/bin/python3\n\"\"\"Maintain one Wi-Fi route for the authorized laptop, learned from ARP.\n\nNever modifies NetworkManager profiles, default routes, FC routes, DNS or sysctls.\nOnly owned proto-186 /32 routes with metric 9 may be removed by this service.\n\"\"\"\nimport ipaddress\nimport json\nimport logging\nfrom pathlib import Path\nimport select\nimport signal\nimport socket\nimport struct\nimport subprocess\nimport time\n\nIFACE = \"wlP1p1s0\"\nLAPTOP_MAC = bytes.fromhex(\"985f416480ed\")\nJETSON_MAC = bytes.fromhex(\"dc4a9e38207f\")\nPROFILE = \"5bf25dec-358c-4a85-9eed-ee5b19a254ca\"\nPROTO = 186\nMETRIC = 9\nSTATE = Path(\"/run/payload-catch-wifi-peer.json\")\n# Reserved static endpoints must never be taken over by this laptop rule.\nEXCLUDED = {\"192.168.1.1\", \"192.168.1.101\", \"192.168.1.111\",\n            \"192.168.1.123\", \"192.168.1.200\", \"192.168.1.202\", \"192.168.1.203\"}\nSTOP = False\n\ndef command(args):\n    p = subprocess.run(args, capture_output=True, text=True, timeout=5, shell=False)\n    if p.returncode:\n        raise RuntimeError(f\"{args!r}: exit={p.returncode} stderr={p.stderr}\")\n    return p.stdout\n\ndef valid_peer(ip, local):\n    address = ipaddress.IPv4Address(ip)\n    net = ipaddress.IPv4Interface(local).network\n    return (address in net and address not in (net.network_address, net.broadcast_address)\n            and ip != str(ipaddress.IPv4Interface(local).ip) and ip not in EXCLUDED)\n\ndef parse_arp(frame):\n    if len(frame) < 42 or frame[12:14] != b\"\\x08\\x06\":\n        return None\n    hardware, protocol, hlen, plen, operation = struct.unpack(\"!HHBBH\", frame[14:22])\n    if (hardware, protocol, hlen, plen) != (1, 0x800, 6, 4) or operation not in (1, 2):\n        return None\n    if frame[6:12] != frame[22:28]:\n        return None\n    return frame[22:28], socket.inet_ntoa(frame[28:32])\n\ndef route_rows():\n    return json.loads(command([\"ip\", \"-j\", \"-4\", \"route\", \"show\", \"table\", \"main\"]))\n\ndef is_owned(row, peer=None):\n    return (row.get(\"dev\") == IFACE and str(row.get(\"protocol\")) in {str(PROTO), \"bgp\"}\n            and row.get(\"metric\") == METRIC\n            and (peer is None or row.get(\"dst\") in (peer, peer + \"/32\")))\n\nclass RouteManager:\n    def __init__(self):\n        self.peer = None\n        self.last_seen = 0.\n\n    def clear(self):\n        # Protocol, device, prefix and metric all constrain deletion.\n        for row in route_rows():\n            if is_owned(row) and ipaddress.ip_network(row[\"dst\"], strict=False).prefixlen == 32:\n                command([\"ip\", \"-4\", \"route\", \"del\", row[\"dst\"], \"dev\", IFACE,\n                         \"metric\", str(METRIC), \"proto\", str(PROTO)])\n                logging.info(\"Removed own route %s\", row[\"dst\"])\n        self.peer = None\n        self.last_seen = 0.\n        STATE.unlink(missing_ok=True)\n\n    def observe(self, mac, ip, local, now):\n        if self.peer == ip and mac != LAPTOP_MAC:\n            logging.warning(\"Peer IP claimed by another MAC; remove own route\")\n            self.clear()\n            return False\n        if mac != LAPTOP_MAC or not valid_peer(ip, local):\n            return False\n        rows = route_rows()\n        existing = [r for r in rows if r.get(\"dst\") in (ip, ip + \"/32\")]\n        if any(r.get(\"dev\") != IFACE for r in existing):\n            logging.warning(\"Existing host route on another device; refusing %s\", ip)\n            return False\n        if self.peer != ip:\n            self.clear()\n        if not any(is_owned(r, ip) for r in existing):\n            # Add a separately owned route; do not replace existing configuration.\n            command([\"ip\", \"-4\", \"route\", \"add\", ip + \"/32\", \"dev\", IFACE,\n                     \"scope\", \"link\", \"metric\", str(METRIC), \"proto\", str(PROTO)])\n            logging.info(\"Authorized laptop %s -> Wi-Fi\", ip)\n        self.peer, self.last_seen = ip, now\n        STATE.write_text(json.dumps({\"peer\": ip, \"mac\": LAPTOP_MAC.hex(\":\"),\n                                     \"interface\": IFACE, \"last_seen_monotonic\": now})+\"\\n\",\n                         encoding=\"utf-8\")\n        return True\n\n    def expire(self, now):\n        if self.peer and now - self.last_seen > 120:\n            logging.info(\"ARP identity lease expired\")\n            self.clear()\n\ndef context():\n    active = command([\"nmcli\", \"-g\", \"GENERAL.CON-UUID\", \"device\", \"show\", IFACE]).strip()\n    if active != PROFILE:\n        return None\n    rows = json.loads(command([\"ip\", \"-j\", \"address\", \"show\", \"dev\", IFACE]))\n    for row in rows:\n        if row.get(\"address\") != JETSON_MAC.hex(\":\"):\n            raise RuntimeError(\"Unexpected Wi-Fi adapter MAC\")\n        for a in row.get(\"addr_info\", []):\n            if a.get(\"scope\") == \"global\" and a.get(\"family\") == \"inet\":\n                return a[\"local\"] + \"/\" + str(a[\"prefixlen\"])\n    return None\n\ndef request(sock, local, target):\n    own = socket.inet_aton(str(ipaddress.IPv4Interface(local).ip))\n    packet = (b\"\\xff\"*6 + JETSON_MAC + b\"\\x08\\x06\"\n              + struct.pack(\"!HHBBH\", 1, 0x800, 6, 4, 1)\n              + JETSON_MAC + own + bytes(6) + socket.inet_aton(target))\n    sock.send(packet)\n\ndef stop(signum, frame):\n    global STOP\n    STOP = True\n\ndef main():\n    logging.basicConfig(level=logging.INFO, format=\"%(asctime)s %(message)s\")\n    serial = Path(\"/proc/device-tree/serial-number\").read_bytes().rstrip(bytes([0])).decode()\n    if serial != \"1612425621719\":\n        raise RuntimeError(\"Wrong Jetson\")\n    for sig in (signal.SIGTERM, signal.SIGINT):\n        signal.signal(sig, stop)\n    manager = RouteManager()\n    manager.clear()\n    try:\n        with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as sock:\n            sock.bind((IFACE, 0))\n            local = None\n            next_context, next_request = 0., 0.\n            while not STOP:\n                now = time.monotonic()\n                if now >= next_context:\n                    latest = context()\n                    if latest != local:\n                        manager.clear()\n                        local = latest\n                    next_context = now + 5\n                if local and now >= next_request:\n                    target = manager.peer or \"192.168.1.4\"\n                    if valid_peer(target, local):\n                        request(sock, local, target)\n                    next_request = now + 15\n                if select.select([sock], [], [], 1)[0]:\n                    frame = sock.recv(2048)\n                    decoded = parse_arp(frame)\n                    if decoded and local:\n                        manager.observe(*decoded, local, time.monotonic())\n                manager.expire(time.monotonic())\n    finally:\n        manager.clear()\n\nif __name__ == \"__main__\":\n    main()\n"
UNIT_TEXT = "[Unit]\nDescription=Wi-Fi return route for authorized payload-catch laptop\nAfter=NetworkManager.service\nWants=NetworkManager.service\n\n[Service]\nType=simple\nExecStart=/usr/bin/python3 /opt/payload_catch/wifi_ssh_20261026_v3/wifi_peer_route.py\nRestart=on-failure\nRestartSec=5\nTimeoutStopSec=10\nNoNewPrivileges=yes\nProtectSystem=full\nProtectHome=read-only\nPrivateTmp=yes\nStandardOutput=journal\nStandardError=journal\n\n[Install]\nWantedBy=multi-user.target\n"
NAME="payload-catch-wifi-peer.service"
TIMER="payload-catch-wifi-peer-rollback"
BASE=Path("/opt/payload_catch/wifi_ssh_20261026_v3")
UNIT=Path("/etc/systemd/system")/NAME
BACKUP=Path("/home/nvidia/payload_catch_network_backup_20261026_v3")
def run(args, check=True):
    p=subprocess.run(args,capture_output=True,text=True,timeout=20,shell=False)
    if check and p.returncode:
        raise RuntimeError(repr(args)+" exit="+str(p.returncode)+" stderr="+p.stderr)
    return {"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}
def identity():
    serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
    if serial!="1612425621719":
        raise RuntimeError("Wrong device")
def rollback():
    state=json.loads((BACKUP/"transaction.json").read_text(encoding="utf-8"))
    if state["base"]!=str(BASE) or state["unit"]!=str(UNIT):
        raise RuntimeError("Unexpected transaction paths")
    print(json.dumps(run(["systemctl","disable","--now",NAME],False)))
    for p,expected in [(UNIT,state["unit_sha256"]),(BASE/"wifi_peer_route.py",state["helper_sha256"])]:
        if p.exists():
            if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
                raise RuntimeError("Externally changed file; refusing removal: "+str(p))
            p.unlink()
    run(["systemctl","daemon-reload"])
    state["rolled_back"]=True
    (BACKUP/"transaction.json").write_text(json.dumps(state,indent=2)+"\n",encoding="utf-8")
    print("ROLLED_BACK")
def install():
    ast.parse(HELPER)
    if BASE.exists() or UNIT.exists() or BACKUP.exists():
        raise RuntimeError("Destination exists; refusing overwrite")
    if "wlP1p1s0" not in run(["ip","route","get","192.168.1.111"])["stdout"]:
        raise RuntimeError("Unexpected network baseline")
    BACKUP.mkdir(mode=0o700)
    checks={}
    for name,args in {
        "addresses":["ip","-j","address","show"],
        "routes":["ip","-j","-4","route","show","table","main"],
        "services":["systemctl","show","payload-catch-agent","payload-catch-relay","ssh","-p","Id","-p","MainPID","-p","ExecMainStartTimestamp"],
    }.items():
        checks[name]=run(args)
    hashes={}
    for p in list(Path("/etc/NetworkManager/system-connections").glob("*"))+list(Path("/etc/systemd/system").glob("payload-catch*")):
        if p.is_file():
            hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    checks["profile_and_unit_hashes"]=hashes
    (BACKUP/"baseline.json").write_text(json.dumps(checks,indent=2)+"\n",encoding="utf-8")
    BASE.mkdir(mode=0o755)
    (BASE/"wifi_peer_route.py").write_text(HELPER,encoding="utf-8")
    (BASE/"wifi_peer_route.py").chmod(0o644)
    UNIT.write_text(UNIT_TEXT,encoding="utf-8")
    state={"base":str(BASE),"unit":str(UNIT),"unit_sha256":hashlib.sha256(UNIT.read_bytes()).hexdigest(),"helper_sha256":hashlib.sha256((BASE/"wifi_peer_route.py").read_bytes()).hexdigest(),"committed":False}
    (BACKUP/"transaction.json").write_text(json.dumps(state,indent=2)+"\n",encoding="utf-8")
    # Stored root-owned rollback survives client disconnect; exact files only.
    self_path=BACKUP/"transaction.py"
    self_path.write_bytes(Path(__file__).read_bytes())
    try:
        run(["systemd-analyze","verify",str(UNIT)])
        run(["systemctl","daemon-reload"])
        run(["systemd-run","--unit="+TIMER,"--on-active=240s","/usr/bin/python3",str(self_path),"rollback"])
        run(["systemctl","start",NAME])
        time.sleep(2)
        status=run(["systemctl","is-active",NAME])
        route=run(["ip","route","get","192.168.1.4"])
        print(json.dumps({"service":status,"route":route,"backup":str(BACKUP),"rollback_deadline_s":240},indent=2))
    except BaseException:
        rollback()
        raise
def commit():
    state=json.loads((BACKUP/"transaction.json").read_text(encoding="utf-8"))
    run(["systemctl","is-active",NAME])
    run(["systemctl","enable",NAME])
    run(["systemctl","stop",TIMER+".timer"])
    state["committed"]=True
    (BACKUP/"transaction.json").write_text(json.dumps(state,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"committed":True,"rollback_command":"sudo /usr/bin/python3 "+str(BACKUP/"transaction.py")+" rollback"}))
if __name__=="__main__":
    identity()
    {"install":install,"rollback":rollback,"commit":commit}[sys.argv[1]]()
