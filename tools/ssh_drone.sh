#!/usr/bin/env bash
# ssh_drone.sh —— NVIDIA Jetson 机载电脑 SSH 连接助手
#
# 来源：从 drone_package_20260908 提取整理，连接写法出自：
#   - How_to_fly_with_control.md §4.1「SSH连接机载电脑」：
#         ssh nvidia@192.168.1.X -i ~/.ssh/id_ed25519
#   - 从sim2出发_真机集群控制.md §2.1（nvidia@ 免密、地址约定）
#   - 链路连接手册.md（mesh 地址表 / 非交互 SSH 陷阱）
#
# 特点：
#   · 登录用户固定为 nvidia（Jetson），私钥固定 ~/.ssh/id_ed25519，
#     直接拼成 nvidia@<IP> -i <key>，**不依赖、也不修改 ~/.ssh/config**。
#   · 目标可以是 IP，也可以是内置别名 A10 / A08 / drone3 / jetson。
#
# 用法：
#   bash tools/ssh_drone.sh hosts                          # 打印已知主机
#   bash tools/ssh_drone.sh shell 192.168.1.100            # 交互式登录 Jetson
#   bash tools/ssh_drone.sh sh    192.168.1.100 "ip -br addr"
#   bash tools/ssh_drone.sh ping  192.168.1.100
#   bash tools/ssh_drone.sh sync-time 192.168.1.100        # 用本机时间给 Jetson 对时
#   bash tools/ssh_drone.sh init  192.168.1.100            # 一次性：装公钥实现免密
#   bash tools/ssh_drone.sh copy  本地文件 192.168.1.100:/tmp/
#   # 别名同样可用：
#   bash tools/ssh_drone.sh shell A10                       # A10 = 192.168.1.101
#
# 也可用环境变量指定默认目标（省得每次写 IP）：
#   export DRONE_IP=192.168.1.100
#   bash tools/ssh_drone.sh shell            # 不给参数就用 $DRONE_IP
#
# 可覆盖：
#   DRONE_USER  登录用户（默认 nvidia）
#   DRONE_KEY   私钥路径（默认 ~/.ssh/id_ed25519）
#   DRONE_IP    未显式给主机时使用的默认 IP
set -euo pipefail

DRONE_USER="${DRONE_USER:-nvidia}"
DRONE_KEY="${DRONE_KEY:-$HOME/.ssh/id_ed25519}"
DRONE_IP="${DRONE_IP:-}"

# ── 内置主机别名 → IP（机载电脑均为 mesh 网段 192.168.1.0/24）──
#    .101 A10(drone1)  .102 A08(drone2)  .103 drone3(预留)
#    .100 地面站 sim2   .123 笔记本
#    .200/.202/.203 是 LQ-MESH 电台自用，机载电脑严禁用 .20x
declare -A HOST_IP=(
  [A10]="192.168.1.101"
  [A08]="192.168.1.102"
  [drone3]="192.168.1.103"
  [jetson]="192.168.155.186"  # 单机 Jetson（WiFi，2026-10 实测；DHCP 会变，可用 DRONE_IP 覆盖）
)

# 解析主机参数：别名→IP；已是 IP/主机名则原样返回
resolve_ip() {
  local h="${1:-}"
  if [ -z "$h" ]; then
    if [ -n "$DRONE_IP" ]; then echo "$DRONE_IP"; return; fi
    echo "错误：未指定主机，且未设置 DRONE_IP" >&2
    echo "提示：bash $0 shell <IP>   或   export DRONE_IP=<IP>" >&2
    exit 1
  fi
  echo "${HOST_IP[$h]:-$h}"
}

# 统一构造远程目标：nvidia@<ip>，并附上私钥
ssh_target() { echo "${DRONE_USER}@$(resolve_ip "$1")"; }

usage() { sed -n '2,42p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

cmd_hosts() {
  printf '%-8s %-16s %s\n' "别名" "IP" "说明"
  printf '%-8s %-16s %s\n' "A10"    "${HOST_IP[A10]}"    "drone1 机载电脑 (Jetson)"
  printf '%-8s %-16s %s\n' "A08"    "${HOST_IP[A08]}"    "drone2 机载电脑 (Jetson)"
  printf '%-8s %-16s %s\n' "drone3" "${HOST_IP[drone3]}" "drone3（预留）"
  printf '%-8s %-16s %s\n' "jetson" "${HOST_IP[jetson]}" "单机 Jetson（默认）"
  echo
  echo "登录用户：$DRONE_USER"
  echo "私钥：    $DRONE_KEY"
  [ -n "$DRONE_IP" ] && echo "默认 IP： $DRONE_IP"
  echo
  echo "连接样式： ssh ${DRONE_USER}@<IP> -i ${DRONE_KEY}"
  echo "注意：mesh 网段 192.168.1.0/24；机载电脑一律 .10x，严禁 .20x（电台占用）。"
}

# 一次性免密：ssh-copy-id（需要输入一次 nvidia 的密码）
cmd_init() {
  local ip; ip="$(resolve_ip "${1:-}")"
  [ -f "$DRONE_KEY" ] || { echo "私钥不存在：$DRONE_KEY" >&2; echo "先生成：ssh-keygen -t ed25519 -f $DRONE_KEY" >&2; exit 1; }
  echo "==> ssh-copy-id ${DRONE_USER}@${ip}  （会提示输入一次密码）"
  ssh-copy-id -i "$DRONE_KEY.pub" "${DRONE_USER}@${ip}"
  echo "==> 完成。之后以下命令免密："
  echo "    ssh ${DRONE_USER}@${ip} -i ${DRONE_KEY}"
}

cmd_shell() {
  local ip; ip="$(resolve_ip "${1:-}")"; shift || true
  exec ssh -i "$DRONE_KEY" -o ConnectTimeout=20 "${DRONE_USER}@${ip}" "$@"
}

cmd_sh() {
  local ip; ip="$(resolve_ip "${1:-}")"; shift || true
  [ $# -gt 0 ] || { echo "用法: $0 sh <IP|别名> \"<命令>\"" >&2; exit 1; }
  ssh -i "$DRONE_KEY" -o ConnectTimeout=20 -o BatchMode=yes "${DRONE_USER}@${ip}" "$@"
}

# 现场对时：Jetson 无 RTC 电池，开机时间是 1970；用本机时间当时间源。
cmd_sync_time() {
  local ip; ip="$(resolve_ip "${1:-}")"
  echo "==> ${ip} 对时 → $(date '+%Y-%m-%d %H:%M:%S')"
  ssh -i "$DRONE_KEY" -o ConnectTimeout=20 -o BatchMode=yes "${DRONE_USER}@${ip}" \
    "sudo date -s \"$(date '+%Y-%m-%d %H:%M:%S')\""
}

cmd_ping() {
  local ip; ip="$(resolve_ip "${1:-}")"
  if ping -c 1 -W 2 "$ip" >/dev/null 2>&1; then
    echo "  [ok]   ${DRONE_USER}@${ip}"
  else
    echo "  [FAIL] ${DRONE_USER}@${ip}  —— 检查 mesh 链路 / 电台供电 / IP 冲突"
    exit 1
  fi
}

cmd_copy() {
  local src="${1:?用法: $0 copy <本地文件> <IP|别名>:<远程路径>}"
  local dst="${2:?用法: $0 copy <本地文件> <IP|别名>:<远程路径>}"
  local host="${dst%%:*}"; local path="${dst#*:}"
  local ip; ip="$(resolve_ip "$host")"
  scp -i "$DRONE_KEY" -o ConnectTimeout=20 "$src" "${DRONE_USER}@${ip}:${path}"
}

main() {
  local sub="${1:-}"; shift || true
  case "$sub" in
    hosts)                cmd_hosts ;;
    init)                 cmd_init "$@" ;;
    shell)                cmd_shell "$@" ;;
    sh|exec)              cmd_sh "$@" ;;
    sync-time|sync)       cmd_sync_time "$@" ;;
    ping)                 cmd_ping "$@" ;;
    copy|scp)             cmd_copy "$@" ;;
    ""|-h|--help|help)    usage ;;
    *) echo "未知子命令：$sub" >&2; usage ;;
  esac
}

main "$@"
