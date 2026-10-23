from pathlib import Path
work=Path(__file__).resolve().parent
remote=(work/"repair_peer_route_remote_20261021.py").read_text(encoding="utf-8")
remote=remote.replace("192.168.1.2","192.168.1.124").replace("peer_route_backup.json","qgc_peer_route_backup.json")
(work/"repair_qgc_route_remote_20261021.py").write_text(remote,encoding="utf-8")
client=(work/"repair_ssh_client_20261021.py").read_text(encoding="utf-8")
client=client.replace("repair_peer_route_remote_20261021.py","repair_qgc_route_remote_20261021.py")
client=client.replace(chr(34)+"route_"+chr(34),chr(34)+"qgc_route_"+chr(34))
client=client.replace("normal_ssh_verified_20261021.json","qgc_normal_ssh_verified_20261021.json").replace("192.168.1.2","192.168.1.124")
(work/"repair_qgc_ssh_client_20261021.py").write_text(client,encoding="utf-8")
