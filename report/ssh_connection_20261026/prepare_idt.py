from pathlib import Path
work=Path(__file__).resolve().parent
text=(work/"retry_a.py").read_text(encoding="utf-8")
text=text.replace("192.168.1.4","192.168.154.192").replace("range(4)","range(152,156)").replace("{(1,1),(1,4)}","{(152,1),(154,192)}")
(work/"retry_idt.py").write_text(text,encoding="utf-8")
