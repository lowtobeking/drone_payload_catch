from pathlib import Path
p=Path("payload_catch/fcu_params.py")
s=p.read_text(encoding="utf-8")
old="add('独立 RC 输入启用',int(p['COM_RC_IN_MODE'])==0)"
new="add('独立 RC 输入模式',int(p['COM_RC_IN_MODE']) in (0,3))"
assert s.count(old)==1
p.write_text(s.replace(old,new),encoding="utf-8")
p=Path("tools/test_fcu_params.py")
s=p.read_text(encoding="utf-8")
needle="check('indoor 合规无 fail', not has_fail(F.check_params(indoor, 'indoor')))"
assert s.count(needle)==1
s=s.replace(needle,needle+"\ncheck('A 的 RC 模式 3 可用，仍需实时 RC 检查',\n      not has_fail(F.check_params({**indoor, 'COM_RC_IN_MODE': 3}, 'indoor')))",1)
p.write_text(s,encoding="utf-8")
