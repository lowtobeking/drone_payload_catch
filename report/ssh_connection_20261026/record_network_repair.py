from pathlib import Path
import json
work=Path(__file__).resolve().parent
root=work.parents[1]
doc=work/"A机WiFi_SSH修复与回退_20261026.md"
(root/"docs"/doc.name).write_bytes(doc.read_bytes())
p=root/"MEMORY.md"
text=p.read_text(encoding="utf-8")
text += """

## 2026-10-26 有线恢复后修复Wi-Fi SSH（最新）

用户授权接电脑网线配置Wi-Fi，要求保留飞控功能。有线eno1=10.41.10.100身份serial1612425621719已核对，boot3f7e18cb。电脑Wi-Fi保持DHCP .4/22与自动DNS；只临时给电脑有线加10.41.10.50（无gateway/DNS、保留原.123），已删除恢复。A当前Wi-Fi .5/22；原Mesh /24抢回程是已确认原因，内核ip rule不支持。新增payload-catch-wifi-peer.service（enabled/active，v3）按电脑Wi-Fi MAC98:5f:41:64:80:ed被动ARP识别并维护唯一proto186/metric9 /32 Wi-Fi路由，支持同网卡DHCP地址变化，保留其他路由/配置；识别不是身份安全边界，SSH仍严格旧主机密钥。9项离线测试通过，明确强制Wi-Fi接口SSH及普通SSH均成功。电脑实际DHCP变化/整机重启尚未实测；A自身DHCP地址也可能变化。

用户确认飞控原网线接回后10.41.10.2 ping2/2，12秒被动接收5844个飞控UDP包（含8888）；原Agent1155/relay1156未重启，7个原配置文件哈希和原路由不变。没有飞控参数/指令、解锁/PWM/系统重启/原服务重启。原本Masked的microxrce-agent/mav-relay不应启动；实际运行的是managed路径，保持原状。current仍b4453dd。原始验收日志report/ssh_connection_20261026，说明docs/A机WiFi_SSH修复与回退_20261026.md。精确撤销：sudo python3 /home/nvidia/payload_catch_network_backup_20261026_v3/transaction.py rollback。旧两次失败安装已回退，最终仅v3有效；验证后自动回退计时器inactive。退出SSH不停止保留的网络辅助服务。没有其他常驻采集或SSH会话。
"""
p.write_text(text,encoding="utf-8")
print("Saved network repair documentation and handoff.")
