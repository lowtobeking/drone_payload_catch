"""Inspect commit candidates; never emit credential values."""
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
def git(*args):
    p=subprocess.run(["git",*args],cwd=ROOT,capture_output=True,timeout=40)
    if p.returncode:
        raise RuntimeError(p.stderr.decode(errors="replace"))
    return p.stdout
names=sorted(set(x.decode() for x in git("ls-files","-z","--modified","--others","--exclude-standard").split(b"\0") if x))
patterns={
"private_key":r"-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----",
"github_token":r"\b(?:gh[pousr]_[A-Za-z0-9_]{25,}|github_pat_[A-Za-z0-9_]{30,})\b",
"credential_assignment":r"(?i)\b(?:password|passwd|wifi_password|token|secret)\s*[:=]\s*['\"]([^'\"]{5,})['\"]",
}
findings=[]
errors=[]
counts=Counter()
inventory=[]
for name in names:
    path=ROOT/name
    data=path.read_bytes()
    inventory.append({"path":name,"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()})
    try:
        text=data.decode("utf-8")
    except UnicodeDecodeError:
        errors.append({"path":name,"error":"not UTF-8"})
        continue
    for label,pattern in patterns.items():
        for match in re.finditer(pattern,text):
            findings.append({"path":name,"line":text.count("\n",0,match.start())+1,"kind":label})
    try:
        if path.suffix==".py":
            ast.parse(text,name)
            counts["python"]+=1
        elif path.suffix==".sh":
            p=subprocess.run(["/bin/bash","-n",str(path)],capture_output=True,timeout=10)
            if p.returncode: raise ValueError(p.stderr.decode(errors="replace"))
            counts["bash"]+=1
        elif path.suffix==".json":
            json.loads(text)
            counts["json"]+=1
        elif path.suffix in (".yaml",".yml"):
            import yaml
            yaml.safe_load(text)
            counts["yaml"]+=1
    except (SyntaxError,ValueError) as exc:
        errors.append({"path":name,"error":str(exc)})
result={"candidates":len(names),"total_bytes":sum(x["bytes"] for x in inventory),"parse_counts":dict(counts),
"errors":errors,"credential_candidates":findings,"inventory":inventory}
(OUT/"snapshot_audit.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in result.items() if k!="inventory"},indent=2))
raise SystemExit(1 if errors else 0)
