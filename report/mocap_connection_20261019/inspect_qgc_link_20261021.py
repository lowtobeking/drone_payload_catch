import subprocess, shutil
for args in (
 ['ip','route','get','192.168.1.123'],
 ['ip','neigh','show','dev','enx00e04c680065'],
 ['ping','-c','2','-W','1','192.168.1.123'],
 ['sudo','-n','journalctl','-u','payload-catch-mav-relay.service','--no-pager','-n','4']):
    r=subprocess.run(args,capture_output=True,text=True,timeout=10)
    print(args,'EXIT',r.returncode,r.stdout,r.stderr,flush=True)
if shutil.which('tcpdump'):
    r=subprocess.run(['sudo','-n','timeout','8','tcpdump','-n','-i','any','-c','25','udp port 14550'],capture_output=True,text=True,timeout=12)
    print('PACKET_HEADERS',r.returncode,r.stdout,r.stderr)
