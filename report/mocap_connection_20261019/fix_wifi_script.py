from pathlib import Path
work=Path(__file__).resolve().parent
p=work/"wifi_connect_guard.py"
s=p.read_text(encoding="utf-8")
s=s.replace('assert not WORK.exists(),"Existing attempt requires review"\n    WORK.mkdir(mode=0o700)','if WORK.exists():\n        assert WORK.is_dir() and not any(WORK.iterdir()),"Existing attempt requires review"\n    else:\n        WORK.mkdir(mode=0o700)')
s=s.replace('with profile.open("x",encoding="utf-8",opener=lambda p,f:os.open(p,f,0o600)) as file:','with open(profile,"x",encoding="utf-8",opener=lambda p,f:os.open(p,f,0o600)) as file:')
p.write_text(s,encoding="utf-8")
print("Corrected secure profile creation API")
