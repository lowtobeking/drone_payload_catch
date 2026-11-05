from pathlib import Path
p=Path("tools/a_solo_preflight.py")
s=p.read_text(encoding="utf-8")
old='        parameters = live_parameters(args.mavlink)\n'
new='''        if args.mavlink == "passive:eno1":
            import os
            import subprocess
            env = dict(os.environ, PYTHONPATH=str(ROOT) + ":" + os.environ.get("PYTHONPATH", ""))
            proc = subprocess.run(["sudo", "-n", "/usr/bin/env",
                "PYTHONPATH=" + env["PYTHONPATH"],
                "/usr/bin/python3", str(ROOT / "tools/a_solo_parameters_passive.py")],
                cwd=ROOT, timeout=65, capture_output=True, text=True,
                encoding="utf-8", errors="replace", shell=False)
            if proc.returncode:
                raise RuntimeError("passive A parameter read failed: " + proc.stderr.strip())
            parameters = json.loads(proc.stdout)
        else:
            parameters = live_parameters(args.mavlink)
'''
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")
