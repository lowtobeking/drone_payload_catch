import pathlib, subprocess
for args in (
 ['ip','route','get','192.168.1.2'],
 ['ip','route','get','192.168.1.123'],
 ['ip','route','get','192.168.1.111'],
 ['systemctl','is-active','payload-catch-agent.service','payload-catch-mav-relay.service'],
 ['systemctl','show','payload-catch-mav-relay.service','-p','ExecStart'],
 ['sudo','-n','journalctl','-u','payload-catch-mav-relay.service','--no-pager','-n','3']):
    r=subprocess.run(args,capture_output=True,text=True,timeout=10)
    print('CMD',args,'EXIT',r.returncode,r.stdout,r.stderr,flush=True)
probe=pathlib.Path('/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021/planeA_probe')
if not probe.is_file(): raise RuntimeError('Missing known read-only VRPN probe')
r=subprocess.run([str(probe),'192.168.1.4'],cwd=str(probe.parent),capture_output=True,text=True,timeout=18)
print('VRPN_READ_ONLY',r.returncode,r.stdout,r.stderr,flush=True)
raise SystemExit(r.returncode)
