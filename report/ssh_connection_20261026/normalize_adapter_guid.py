from pathlib import Path
path=Path(__file__).with_name("trial_historical_ip.ps1")
text=path.read_text(encoding="utf-8").replace("$adapter.InterfaceGuid.ToString() -ne", "$adapter.InterfaceGuid.ToString().Trim('{}') -ne")
path.write_text(text,encoding="utf-8")
