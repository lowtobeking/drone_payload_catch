from pathlib import Path
from pyulog import ULog
root=Path(__file__).resolve().parents[1]
run=Path(__import__("json").loads((root/"report/a_solo_gazebo_20261019/latest_run.json").read_text())["path"])
log=next((run/"log").rglob("*.ulg"))
u=ULog(str(log))
for name in ("vehicle_status","vehicle_local_position","vehicle_local_position_groundtruth",
             "trajectory_setpoint","vehicle_thrust_setpoint","actuator_motors",
             "vehicle_visual_odometry","timesync_status","takeoff_status","vehicle_land_detected"):
    try:d=u.get_dataset(name).data
    except (KeyError,IndexError,ValueError):continue
    print("\nDATA",name,"n",len(d["timestamp"]))
    keys=[k for k in d if k in ("timestamp","arming_state","nav_state","z","vz","takeoff_state",
        "landed","observed_offset","estimated_offset","round_trip_time") or
        any(k.startswith(p) for p in ("velocity[","position[","control[","xyz["))
        or k=="timestamp_sample"]
    print("RANGES",{k:(float(d[k].min()),float(d[k].max())) for k in keys if k!="timestamp"})
    step=max(1,len(d["timestamp"])//12)
    for i in range(0,len(d["timestamp"]),step):
        print({k:round(float(d[k][i]),4) for k in keys})
