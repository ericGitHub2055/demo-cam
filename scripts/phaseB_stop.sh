#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker/compose.ros2.yaml"
SERVICE="ros2"
PID_DIR="/work/demo-cam/outputs/phaseB/pids"

cexec() {
  docker compose -f "$COMPOSE_FILE" exec -T "$SERVICE" bash -lc "$*"
}

echo "[phaseB_stop] Attempting stop by pid files (if any)..."
cexec "shopt -s nullglob; \
      for f in '$PID_DIR'/*.pid; do \
        p=\$(cat \"\$f\" 2>/dev/null || true); \
        echo \"  stopping \$f pid=\$p\"; \
        if [ -n \"\$p\" ] && kill -0 \"\$p\" 2>/dev/null; then kill \"\$p\" 2>/dev/null || true; fi; \
      done; \
      rm -f '$PID_DIR'/*.pid 2>/dev/null || true"

echo "[phaseB_stop] Fallback stop by process patterns..."
cexec "\
pkill -f '/opt/ros/jazzy/lib/usb_cam/usb_cam_node_exe' || true; \
pkill -f 'demo_cam_detect.*detector_node' || true; \
pkill -f 'demo_cam_detect.*mjpeg_bridge' || true; \
pkill -f 'ros2 run usb_cam usb_cam_node_exe' || true; \
pkill -f 'ros2 run demo_cam_detect detector_node' || true; \
pkill -f 'ros2 run demo_cam_detect mjpeg_bridge' || true"

echo "[phaseB_stop] Remaining matching processes (should be empty):"
cexec "ps -ef | egrep 'usb_cam_node_exe|mjpeg_bridge|detector_node|demo_cam_detect' | egrep -v egrep || true"

echo "[phaseB_stop] Done."
