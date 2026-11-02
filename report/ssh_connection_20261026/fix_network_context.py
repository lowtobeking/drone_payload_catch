from pathlib import Path
import json
work=Path(__file__).resolve().parent
helper=(work/"wifi_peer_route.py").read_text(encoding="utf-8")
helper=helper.replace('["ip", "-j", "-4", "address", "show", "dev", IFACE]', '["ip", "-j", "address", "show", "dev", IFACE]')
helper=helper.replace('if a.get("scope") == "global":', 'if a.get("scope") == "global" and a.get("family") == "inet":')
(work/"wifi_peer_route.py").write_text(helper,encoding="utf-8")
p=work/"network_transaction.py"
text=p.read_text(encoding="utf-8")
start=text.index("HELPER = ")
end=text.index("\nUNIT_TEXT",start)
text=text[:start]+"HELPER = "+json.dumps(helper)+text[end:]
text=text.replace("wifi_ssh_20261026","wifi_ssh_20261026_v2").replace("payload_catch_network_backup_20261026","payload_catch_network_backup_20261026_v2")
p.write_text(text,encoding="utf-8")
stage=work/"stage_network_fix.py"
text=stage.read_text(encoding="utf-8").replace("payload_catch_network_transaction_20261026.py","payload_catch_network_transaction_20261026_v2.py")
stage.write_text(text,encoding="utf-8")
