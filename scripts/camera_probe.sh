#!/usr/bin/env bash
set -euo pipefail

DEV="${1:-/dev/video0}"

echo "[INFO] Hostname: $(hostname)"
echo "[INFO] OS: $(cat /etc/os-release | grep PRETTY_NAME || true)"
echo "[INFO] Kernel: $(uname -a)"
echo

echo "[INFO] V4L2 devices:"
v4l2-ctl --list-devices || true
echo

if [[ ! -e "$DEV" ]]; then
  echo "[ERR] Device not found: $DEV"
  echo "[HINT] Run: ls -l /dev/video*"
  exit 1
fi

echo "[INFO] Device node: $DEV"
ls -l "$DEV"
echo

echo "[INFO] Supported formats (ext):"
v4l2-ctl -d "$DEV" --list-formats-ext | sed -n '1,220p'
echo

echo "[INFO] Quick recommendation:"
echo "- For Logitech C920S, prefer MJPG at 1280x720@30 if available."
echo "- If 1080p30 MJPG is available, try 1920x1080@30 but expect higher CPU for software encode."
echo
