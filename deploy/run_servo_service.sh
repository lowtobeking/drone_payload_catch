#!/usr/bin/env bash
# Administrator-provisioned root service only. Never source a user-writable tree.
set -eo pipefail
[[ $EUID == 0 ]] || { echo 'root service required'; exit 1; }
[[ "${FLEET_SERVO_RELEASE:-}" == /opt/payload_catch/* && "${PX4_MSGS_SETUP:-}" == /opt/payload_catch/* && "${FLEET_CONFIG:-}" == /etc/payload_catch/* ]] || { echo 'Use protected /opt and /etc paths'; exit 1; }
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
/usr/bin/python3 - "$FLEET_SERVO_RELEASE" "$PX4_MSGS_SETUP" "$FLEET_CONFIG" <<'PY'
import os,stat,sys
from pathlib import Path
def check(path):
    path=path.resolve(strict=True)
    for parent in (path,*path.parents):
        s=parent.stat()
        if s.st_uid!=0 or s.st_mode&0o022:raise SystemExit('root ownership / write permissions unsafe: '+str(parent))
for value in sys.argv[1:]:
    p=Path(value);check(p)
    tree=p if p.is_dir() else (p.parent if value.endswith('setup.bash') else None)
    if tree is not None:
        for child in tree.rglob('*'):check(child)
PY
source /opt/ros/humble/setup.bash
source "$PX4_MSGS_SETUP"
source "$FLEET_SERVO_RELEASE/ws/install/setup.bash"
release_id="$(python3 "$FLEET_SERVO_RELEASE/src/tools/fleet_deploy.py" verify "$FLEET_SERVO_RELEASE/src")"
if [[ "${1:-run}" == restore ]]; then
  exec python3 -m payload_catch.servo_pwm restore
fi
python3 "$FLEET_SERVO_RELEASE/src/tools/fleet_configure.py" "$FLEET_CONFIG" --role a --out /run/payload-servo/network --check-host
source /run/payload-servo/network/network.env
servo_path="$(python3 - "$FLEET_CONFIG" <<'PY'
import sys
from payload_catch.fleet_runtime import load_fleet
_,p,_=load_fleet(sys.argv[1])
for x in (p,*p.parents):
    s=x.stat()
    if s.st_uid!=0 or s.st_mode&0o022:raise SystemExit('unsafe servo config permissions')
print(p)
PY
)"
exec ros2 run payload_catch payload_servo --ros-args --params-file "$servo_path" -p owner:=0 -p release_id:="$release_id"
