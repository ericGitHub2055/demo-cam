#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker/compose.ros2.yaml"
SERVICE="ros2"

IMAGE_TOPIC="/image_raw"
VIZ_TOPIC="/detections_image"
DET_TOPIC="/detections"
MJPEG_PORT="8080"

cexec() {
  docker compose -f "$COMPOSE_FILE" exec -T "$SERVICE" bash -lc "$*"
}

echo "[healthcheck] Container status:"
docker compose -f "$COMPOSE_FILE" ps

echo "[healthcheck] Checking ROS2 topics publisher count..."

check_pub () {
  local topic="$1"
  if cexec "source /opt/ros/jazzy/setup.bash && ros2 topic info '$topic' -v" >/tmp/_topic_info.txt 2>&1; then :; fi
  if grep -qE "Publisher count: [1-9]" /tmp/_topic_info.txt; then
    echo "  [OK] $topic has publisher"
  else
    echo "  [FAIL] $topic has NO publisher"
    echo "  ---- ros2 topic info $topic -v ----"
    cat /tmp/_topic_info.txt
    echo "  ----------------------------------"
    return 1
  fi
}

check_pub "$IMAGE_TOPIC"
check_pub "$VIZ_TOPIC"
check_pub "$DET_TOPIC"

echo "[healthcheck] Message receipt check (echo --once, 3s timeout)..."

check_once () {
  local topic="$1"
  # Run inside container; if one message arrives within 3s -> OK
  if cexec "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 topic echo '$topic' --once >/dev/null 2>&1"; then
    echo "  [OK]  $topic received at least 1 msg"
  else
    echo "  [FAIL] $topic did NOT produce msg in 3s"
    return 1
  fi
}

check_once "$IMAGE_TOPIC"
check_once "$VIZ_TOPIC"
check_once "$DET_TOPIC"

echo "[healthcheck] Checking MJPEG endpoint (TCP + HTTP header)..."

# TCP connectivity
if cexec "python3 - <<'PY'
import socket, sys
port=int('${MJPEG_PORT}')
s=socket.socket()
s.settimeout(2)
try:
    s.connect(('127.0.0.1', port))
    print('OK')
except Exception as e:
    print('FAIL', e)
    sys.exit(1)
finally:
    s.close()
PY" >/tmp/_tcp.txt 2>&1; then
  echo "  [OK] TCP port ${MJPEG_PORT} accepts connections"
else
  echo "  [FAIL] TCP port ${MJPEG_PORT} not accepting connections"
  cat /tmp/_tcp.txt
  exit 1
fi

# HTTP header check (do not wait for first JPEG frame)
if cexec "command -v curl >/dev/null 2>&1"; then
  # We only need to see HTTP 200 + Content-Type multipart...
  if cexec "curl -sv --max-time 3 http://127.0.0.1:${MJPEG_PORT}/stream.mjpg 2>&1 | head -n 30" >/tmp/_hdr.txt 2>&1; then :; fi
  if grep -qE "HTTP/.* 200" /tmp/_hdr.txt && grep -qi "Content-Type: multipart/x-mixed-replace" /tmp/_hdr.txt; then
    echo "  [OK] MJPEG HTTP headers look correct"
  else
    echo "  [FAIL] MJPEG HTTP headers not as expected"
    echo "  ---- curl -sv (first 30 lines) ----"
    cat /tmp/_hdr.txt
    echo "  ----------------------------------"
    exit 1
  fi
else
  echo "  [WARN] curl not available in container; TCP check passed, skipping HTTP header check"
fi

echo "[healthcheck] Summary: ALL GREEN"
echo "  MJPEG URL: http://<PI_IP>:${MJPEG_PORT}/stream.mjpg"
