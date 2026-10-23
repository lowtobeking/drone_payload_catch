import subprocess, pathlib
for args in (
    ['ip','-4','address','show'],
    ['ip','route','get','192.168.1.203'],
    ['ip','route','get','192.168.1.124'],
    ['systemctl','show','payload-catch-mav-relay.service','-p','ExecStart'],
    ['nmcli','-g','ipv4.routes','connection','show','5bf25dec-358c-4a85-9eed-ee5b19a254ca'],
    ['ping','-I','wlP1p1s0','-c','2','-W','1','192.168.1.203'],
    ['ip','neigh','show','to','192.168.1.203']):
    r=subprocess.run(args,capture_output=True,text=True,timeout=12)
    print(args,'EXIT',r.returncode,r.stdout,r.stderr,flush=True)
p=pathlib.Path('/home/nvidia/payload_catch_deploy/host/runtime.yaml')
print('PROFILE',p.read_text(encoding='utf-8'))
