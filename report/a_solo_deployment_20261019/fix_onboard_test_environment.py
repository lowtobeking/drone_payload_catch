from pathlib import Path
root=Path(__file__).resolve().parents[2]
path=root/"tools/run_a_solo_isolated.sh"
s=path.read_text(encoding="utf-8")
old="    source ./env.sh >/dev/null"
new="""    if [[ -f ./env.sh ]]; then
        source ./env.sh >/dev/null
    else
        set +u
        source /opt/ros/humble/setup.bash
        : "${PX4_MSGS_SETUP:?explicit independent message overlay required}"
        source "$PX4_MSGS_SETUP"
        source "$PAYLOAD_CATCH_TEST_ROOT/../ws/install/setup.bash"
        set -u
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    fi"""
assert s.count(old)==1
s=s.replace(old,new)
old='    ROS_LOCALHOST_ONLY=1 bash "$0" --worker'
new='    PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-}" ROS_LOCALHOST_ONLY=1 bash "$0" --worker'
assert s.count(old)==1
path.write_text(s.replace(old,new),encoding="utf-8",newline="\n")
p=root/"report/a_solo_deployment_20261019/prepare_deployment.py"
s=p.read_text(encoding="utf-8")
s=s.replace("from tools.fleet_deploy import bundle,read_bundle","from tools.fleet_deploy import bundle,read_bundle,make_manifest,snapshot")
old='''if target.exists():
    files,manifest=read_bundle(target)
    release=manifest["release_id"]
else:'''
new='''if target.exists() and read_bundle(target)[1] != make_manifest(snapshot(root)):
    archive=report/"bundles"
    archive.mkdir(exist_ok=True)
    old_release=read_bundle(target)[1]["release_id"]
    target.rename(archive/(old_release+".tar.gz"))
if target.exists():
    files,manifest=read_bundle(target)
    release=manifest["release_id"]
else:'''
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")
