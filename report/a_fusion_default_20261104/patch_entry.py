from pathlib import Path
p=Path("deploy/run_a_solo.sh")
s=p.read_text(encoding="utf-8")
old='mode="$4"\n'
new='''mode="$4"
if [[ "$mode" == execute && "${A_SOLO_MOCAP_READY:-0}" != 1 ]]; then
    echo 'Indoor A execution requires deploy/run_a_solo_mocap.sh' >&2
    exit 1
fi
'''
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")
p=Path("deploy/run_a_solo_mocap.sh")
s=p.read_text(encoding="utf-8")
old='bash "$root/deploy/run_a_solo.sh" "$config" "$receipt" "$profile" execute\n'
new='A_SOLO_MOCAP_READY=1 bash "$root/deploy/run_a_solo.sh" "$config" "$receipt" "$profile" execute\n'
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")
