from pathlib import Path
import fcntl
import hashlib
import json
import os
import platform
import shutil
import socket
import subprocess
base=Path("/home/nvidia/payload_catch_deploy")
def read(path):
    try:return Path(path).read_text(encoding="utf-8",errors="replace").strip().strip("\0")
    except (OSError,UnicodeError):return None
def digest(path):
    try:return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:return None
def command(args):
    p=subprocess.run(args,timeout=10,capture_output=True,text=True,encoding="utf-8",errors="replace")
    return dict(returncode=p.returncode,stdout=p.stdout.strip(),stderr=p.stderr.strip())
locks={}
for name in ("mission.lock","operation.lock"):
    try:
        with (base/name).open("r+") as f:
            fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        locks[name]=True
    except OSError:locks[name]=False
processes=[]
for item in Path("/proc").iterdir():
    if not item.name.isdigit() or int(item.name)==os.getpid():continue
    try:args=(item/"cmdline").read_bytes().replace(b"\0",b" ").decode("utf-8",errors="replace").strip()
    except OSError:continue
    if any(token in args for token in ("a_solo_validation","a_solo_control.py","catch_real_a_launch","catch_real_b_launch","catch_stack_launch","payload_servo_node","safety_supervisor","release_guard","a_node","b_node","run_a_solo.sh","run_fleet.sh")):
        processes.append(dict(pid=int(item.name),argv=args))
pwm={}
for chip in Path("/sys/class/pwm").glob("pwmchip*"):
    if "32c0000" in str(chip.resolve()):
        pwm["chip"]=str(chip.resolve())
        for name in ("enable","duty_cycle","period","polarity"):
            pwm[name]=read(chip/"pwm0"/name)
profile=base/"host/runtime.yaml"
data=dict(hostname=socket.gethostname(),architecture=platform.machine(),
    os_release=read("/etc/os-release"),l4t=read("/etc/nv_tegra_release"),
    serial=read("/proc/device-tree/serial-number"),
    boot_id=read("/proc/sys/kernel/random/boot_id"),
    current=str((base/"current").resolve(strict=True)),previous=str((base/"previous").resolve(strict=True)),
    role=read(base/"ROLE"),locks_available=locks,mission_processes=processes,
    free_bytes=shutil.disk_usage(base).free,host_profile_sha256=digest(profile),
    boot_config_sha256=digest("/boot/extlinux/extlinux.conf"),pwm33=pwm,
    services={u:command(["systemctl","show",u,"--property=ActiveState,SubState,MainPID,ExecStart,UnitFileState"]) for u in
        ("payload-catch-agent.service","payload-catch-mav-relay.service","microxrce-agent.service","mav-relay.service")})
print(json.dumps(data,indent=2,ensure_ascii=False),flush=True)
assert data["hostname"]=="nvidia-desktop" and data["architecture"]=="aarch64" and data["role"]=="a"
assert all(locks.values()) and not processes,"Active mission/control process or lock"
assert data["free_bytes"]>300_000_000,"Insufficient build space"
