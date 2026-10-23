from pathlib import Path
import json
import os
targets = ('tools/check_a_full_mission_ros.py', 'tools/run_a_full_mission_isolated.py',
           'tools/run_a_full_mission_isolated.sh', 'tools/check_a_ground_ros.py',
           'tools/check_servo_ros.py', 'tools/check_fleet_ros.py')
found = []
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) == os.getpid():
        continue
    try:
        args = [a.decode(errors='replace') for a in (p/'cmdline').read_bytes().split(b'\0') if a]
    except (OSError, ProcessLookupError):
        continue
    if any(a.endswith(targets) for a in args):
        found.append({'pid': int(p.name), 'args': args})
print(json.dumps({'remaining_task_test_processes': found}, indent=2))
raise SystemExit(bool(found))
