#!/usr/bin/env python3
"""Root only creates netns. Ordinary user runs bounded fake-PX4 acceptance."""
from pathlib import Path
import os
import pwd
import signal
import subprocess
import sys


def main():
    if os.geteuid()!=0:
        raise SystemExit("root required only to create isolated network namespace")
    root=Path(__file__).resolve().parents[1]
    user=pwd.getpwuid(root.stat().st_uid).pw_name
    if user=="root":
        raise SystemExit("repository owner must be an ordinary user")
    script=root/"tools/run_a_solo_isolated.sh"
    subprocess.run(["/bin/bash","-n",str(script)],cwd=root,timeout=5,check=True)
    report=root/"report/a_solo_validation_20261019"
    report.mkdir(parents=True,exist_ok=True)
    logpath=report/"ros_integration.log"
    env=dict(os.environ,PAYLOAD_CATCH_HOST_NETNS=os.readlink("/proc/self/ns/net"),
             PAYLOAD_CATCH_TEST_USER=user,PAYLOAD_CATCH_TEST_ROOT=str(root))
    with logpath.open("w",encoding="utf-8") as logfile:
        process=subprocess.Popen(["unshare","--net","--","/bin/bash",str(script)],
            cwd=root,env=env,start_new_session=True,stdout=logfile,stderr=subprocess.STDOUT)
        print("A_SOLO_ISOLATED_PID="+str(process.pid)+" LOG="+str(logpath),flush=True)
        try:
            code=process.wait(timeout=200)
        except BaseException:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait(timeout=5)
            raise
        finally:
            os.chown(logpath,root.stat().st_uid,root.stat().st_gid)
    print(logpath.read_text(encoding="utf-8"),flush=True)
    return code


if __name__=="__main__":
    raise SystemExit(main())
