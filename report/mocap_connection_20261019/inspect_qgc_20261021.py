import pathlib, subprocess
root=pathlib.Path('/home/nvidia/payload_catch_deploy')
for name in ('host/runtime.yaml','host/runtime_env.sh'):
    p=root/name
    print('FILE',p)
    if p.is_file():
        print(p.read_text(encoding='utf-8'))
for args in (['ip','route','get','192.168.1.124'],['systemctl','is-active','payload-catch-mav-relay.service'],['ss','-unp']):
    r=subprocess.run(args,capture_output=True,text=True,timeout=10)
    print(args,'exit',r.returncode,r.stdout,r.stderr)
