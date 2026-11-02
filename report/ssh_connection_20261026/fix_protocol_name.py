from pathlib import Path
import json
work=Path(__file__).resolve().parent
p=work/"wifi_peer_route.py"
helper=p.read_text(encoding="utf-8").replace('str(row.get("protocol")) == str(PROTO)','str(row.get("protocol")) in {str(PROTO), "bgp"}')
p.write_text(helper,encoding="utf-8")
p=work/"network_transaction.py"
text=p.read_text(encoding="utf-8")
start=text.index("HELPER = ")
end=text.index("\nUNIT_TEXT",start)
text=text[:start]+"HELPER = "+json.dumps(helper)+text[end:]
text=text.replace("20261026_v2","20261026_v3")
p.write_text(text,encoding="utf-8")
p=work/"stage_network_fix.py"
p.write_text(p.read_text(encoding="utf-8").replace("20261026_v2","20261026_v3"),encoding="utf-8")
p=work/"test_wifi_peer_route.py"
text=p.read_text(encoding="utf-8").replace("def test_cleanup_preserves_other_interfaces(self):", "def test_named_protocol_is_owned(self):\n        self.assertTrue(peer.is_owned(dict(dst='192.168.1.4',dev=peer.IFACE,protocol='bgp',metric=9)))\n    def test_cleanup_preserves_other_interfaces(self):")
p.write_text(text,encoding="utf-8")
