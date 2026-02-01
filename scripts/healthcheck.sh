#!/usr/bin/env bash
set -euo pipefail

HOST_IP="${1:-}"
PORT="${2:-5000}"

if [[ -z "$HOST_IP" ]]; then
  echo "Usage: $0 <HOST_MACHINE_IP> [PORT]"
  exit 1
fi

echo "[INFO] Raspberry Pi IP addresses:"
ip -br a | sed -n '1,120p'
echo

echo "[INFO] Testing reachability to host: $HOST_IP"
if ping -c 3 "$HOST_IP" >/dev/null 2>&1; then
  echo "[OK] ping ok"
else
  echo "[WARN] ping failed (ICMP may be blocked by Windows firewall; UDP streaming can still work)"
fi
echo

echo "[INFO] Checking local route to host:"
ip route get "$HOST_IP" || true
echo

echo "[INFO] Notes for Windows host preview:"
echo "- Ensure Windows Firewall allows inbound UDP on port ${PORT} (or allow VLC app)."
echo "- Ensure the stream script uses --host ${HOST_IP} --port ${PORT}."
echo "- If VLC shows no video, edit SDP: set c=IN IP4 ${HOST_IP} if needed."
echo
