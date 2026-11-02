from pathlib import Path
path=Path(__file__).with_name("trial_historical_ip.ps1")
text=path.read_text(encoding="utf-8")
text=text.replace("@('-NoProfile','-File', $PSCommandPath", "@('-NoProfile','-ExecutionPolicy','Bypass','-File', $PSCommandPath")
path.write_text(text,encoding="utf-8")
