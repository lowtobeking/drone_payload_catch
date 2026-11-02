from pathlib import Path
p=Path(__file__).with_name("verify_wifi.py")
text=p.read_text(encoding="utf-8").replace("'pid,ppid,lstart,args'])))))","'pid,ppid,lstart,args']))))")
p.write_text(text,encoding="utf-8")
import ast
ast.parse(text)
namespace={}
exec(compile(ast.Module(body=[n for n in ast.parse(text).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="REMOTE" for t in n.targets)],type_ignores=[]),str(p),"exec"),namespace)
ast.parse(namespace["REMOTE"])
print("Local and remote syntax OK")
