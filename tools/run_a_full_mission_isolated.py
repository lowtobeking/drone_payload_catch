#!/usr/bin/env python3
"""Root creates netns only; all ROS simulation executes as repo owner."""
from pathlib import Path
import os
import pwd
import signal
import subprocess
import sys

def main():
    if os.geteuid()!=0:raise SystemExit('sudo required only for isolated network namespace')
    root=Path(__file__).resolve().parents[1]
    user=pwd.getpwuid(root.stat().st_uid).pw_name
    assert user!='root'
    script=root/'tools/run_a_full_mission_isolated.sh'
    subprocess.run(['/bin/bash','-n',str(script)],check=True,timeout=5)
    env=dict(os.environ,PAYLOAD_CATCH_HOST_NETNS=os.readlink('/proc/self/ns/net'),
             PAYLOAD_CATCH_TEST_USER=user,PAYLOAD_CATCH_TEST_ROOT=str(root))
    report=root/'report/a_full_mission_20261016'
    report.mkdir(parents=True,exist_ok=True)
    os.chown(report,root.stat().st_uid,root.stat().st_gid)
    log=(report/'mission.log').open('w',encoding='utf-8')
    process=subprocess.Popen(['unshare','--net','--','/bin/bash',str(script)],
        cwd=root,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
    print('A_SIMULATION_PID='+str(process.pid),flush=True)
    try:
        code=process.wait(timeout=65)
    except BaseException:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        raise
    finally:
        log.close()
        os.chown(report/'mission.log',root.stat().st_uid,root.stat().st_gid)
    print((report/'mission.log').read_text(),flush=True)
    sys.exit(code)

if __name__=='__main__':main()
