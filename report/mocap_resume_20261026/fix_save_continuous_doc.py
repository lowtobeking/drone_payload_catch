"""Fix documentation-only backslashes in the session archive script."""
import ast
from pathlib import Path
path=Path(__file__).resolve().parent/"save_continuous_session.py"
content=path.read_text(encoding="utf-8")
content=content.replace("C:\\Users\\caoli\\AppData\\Local\\Programs\\Python\\Python312\\python.exe","C:/Users/caoli/AppData/Local/Programs/Python/Python312/python.exe")
content=content.replace("\\\\wsl.localhost\\Ubuntu-22.04\\home\\caolihao\\drone_payload_catch","//wsl.localhost/Ubuntu-22.04/home/caolihao/drone_payload_catch")
ast.parse(content)
path.write_text(content,encoding="utf-8",newline="\n")
print("Documentation paths normalized")
