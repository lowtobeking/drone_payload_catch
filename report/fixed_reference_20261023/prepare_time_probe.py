from pathlib import Path
import ast
WORK=Path(__file__).resolve().parent
OLD=WORK.parent/"mocap_fusion_20261023"
s=(OLD/"receive_parameter_guard_a.py").read_text(encoding="utf-8")
prefix=s[:s.index("def main():")]
prefix=prefix.replace('import backup_parameters_a as backup','import backup_parameters_a as backup')
main='''
def main():
    tap=Tap()
    result={"messages":{}}
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:
            for m in tap.receive():
                result["messages"].setdefault(m.get_type(),m.to_dict())
        token=time.time_ns()
        tap.tx.timesync_send(0,token)
        tap.tx.command_long_send(1,1,512,0,2,0,0,0,0,0,0)
        end=time.monotonic()+5
        while time.monotonic()<end:
            for m in tap.receive():
                if m.get_type() in ("TIMESYNC","SYSTEM_TIME"):
                    result.setdefault("time_responses",[]).append(m.to_dict())
        result["disarmed"]=tap.disarmed()
    finally:
        tap.close()
    save("fc_time_probe.json",result)
    print(json.dumps(result),flush=True)
if __name__=="__main__": main()
'''
ast.parse(prefix+main)
(WORK/"fc_time_probe_a.py").write_text(prefix+main,encoding="utf-8")
s=(OLD/"run_parameter_backup.py").read_text(encoding="utf-8")
s=s.replace('"192.168.1.2"','"192.168.1.123"').replace('"nvidia@192.168.1.4"','"nvidia@192.168.1.4"')
s=s.replace('data = (WORK / "backup_parameters_a.py").read_bytes()',
'''source = (WORK/"fc_time_probe_a.py").read_bytes()
    backup = (WORK/"backup_parameters_a.py").read_bytes()
    data = source''',1)
s=s.replace('"(p/\\\'backup_parameters_a.py\\\').write_bytes(data)\\n")',
''' "(p/'fc_time_probe_a.py').write_bytes(data)\\n"
            "(p/'backup_parameters_a.py').write_bytes("+repr(backup)+")\\n")''')
s=s.replace('remote + "/backup_parameters_a.py"','remote + "/fc_time_probe_a.py", remote+"/logs"')
s=s.replace('remote + "/parameters_backup_passive.json"','remote + "/logs/fc_time_probe.json"')
s=s.replace('("parameters_" +','("time_probe_" +')
# generic exact replacements for run naming and saved file
s=s.replace('name = "parameters_"','name = "time_probe_"')
s=s.replace('local / "parameters_backup.json"','local / "fc_time_probe.json"')
ast.parse(s)
(WORK/"run_fc_time_probe.py").write_text(s,encoding="utf-8")
