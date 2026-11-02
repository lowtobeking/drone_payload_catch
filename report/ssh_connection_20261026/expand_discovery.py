from pathlib import Path
path=Path(__file__).with_name("connect_a.py")
text=path.read_text(encoding="utf-8")
old='["192.168.1."+str(i) for i in range(2,255) if i!=4]'
new='["192.168."+str(net)+"."+str(i) for net in range(4) for i in range(1,255) if (net,i) not in {(1,4),(1,1)}]'
if old not in text:
    raise RuntimeError("Expected scan expression missing")
path.write_text(text.replace(old,new),encoding="utf-8")
