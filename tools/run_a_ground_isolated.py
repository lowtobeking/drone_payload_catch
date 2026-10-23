import json
import os
from pathlib import Path
import subprocess
import sys
import pwd

if os.geteuid()!=0:raise SystemExit("Run with sudo: only creates an isolated network; no hardware I/O")

root=Path(__file__).resolve().parents[1]
account=pwd.getpwuid(root.stat().st_uid).pw_name
if account=='root':raise SystemExit('project must be owned by an ordinary development user')
report=root/'report/ab_optimization_20261016'
subprocess.run(['runuser','-u',account,'--','mkdir','-p',str(report)],check=True)
host=os.readlink('/proc/self/ns/net')
env=dict(os.environ,PAYLOAD_CATCH_HOST_NETNS=host,ROS_LOCALHOST_ONLY='1',PAYLOAD_CATCH_TEST_USER=account,PAYLOAD_CATCH_TEST_ROOT=str(root))
script='''set -e
ip link set lo up
exec runuser -u "$PAYLOAD_CATCH_TEST_USER" -- env PAYLOAD_CATCH_HOST_NETNS="$PAYLOAD_CATCH_HOST_NETNS" ROS_LOCALHOST_ONLY=1 bash -c '
set -e
cd "$PAYLOAD_CATCH_TEST_ROOT"
source ./env.sh >/dev/null
python3 tools/check_a_ground_ros.py >report/ab_optimization_20261016/a_ground_ros.log 2>&1
python3 tools/check_servo_launch.py >report/ab_optimization_20261016/servo_launch.log 2>&1
python3 tools/check_fleet_launch.py >report/ab_optimization_20261016/fleet_launch.log 2>&1
python3 tools/check_servo_ros.py >report/ab_optimization_20261016/servo_ros.log 2>&1
python3 tools/check_fleet_ros.py >report/ab_optimization_20261016/fleet_ros.log 2>&1
'
'''
result=subprocess.run(['unshare','--net','--','bash','-c',script],cwd=root,env=env)
for name in ('a_ground_ros','servo_launch','fleet_launch','servo_ros','fleet_ros'):
    p=report/(name+'.log')
    print(name+':')
    print('\n'.join(p.read_text().splitlines()[-12:]) if p.exists() else 'not run')
sys.exit(result.returncode)
