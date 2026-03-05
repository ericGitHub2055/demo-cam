#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$ROOT_DIR/docker/compose.ros2.yaml"
SERVICE="ros2"

# ---- Config (edit if needed) ----
IMAGE_TOPIC="/image_raw"
VIZ_TOPIC="/detections_image"
MJPEG_PORT="8080"
JPEG_QUALITY="80"
MODEL="/work/demo-cam/models/yolov8n_416.onnx"
IMGSZ="416"
CONF="0.25"
IOU="0.45"

LOG_DIR="/work/demo-cam/outputs/phaseB/logs"
PID_DIR="/work/demo-cam/outputs/phaseB/pids"

# Helpers: run command inside container
cexec() {
  docker compose -f "$COMPOSE_FILE" exec -T "$SERVICE" bash -lc "$*"
}

echo "[phaseB_start] Ensuring container is up..."
docker compose -f "$COMPOSE_FILE" up -d

echo "[phaseB_start] Preparing log/pid dirs in container..."
cexec "mkdir -p '$LOG_DIR' '$PID_DIR'"

# Stop existing if pid files exist
echo "[phaseB_start] Stopping existing processes if any..."
cexec "for f in '$PID_DIR'/*.pid; do \
         [ -f \"\$f\" ] || continue; \
         p=\$(cat \"\$f\" 2>/dev/null || true); \
         if [ -n \"\$p\" ] && kill -0 \"\$p\" 2>/dev/null; then kill \"\$p\" 2>/dev/null || true; fi; \
       done; \
       rm -f '$PID_DIR'/*.pid || true"

echo "[phaseB_start] Starting usb_cam..."
cexec "source /opt/ros/jazzy/setup.bash && \
      nohup ros2 run usb_cam usb_cam_node_exe --ros-args \
        --params-file /work/demo-cam/ros2_ws/src/demo_cam_ros/config/usb_cam.yaml \
      > '$LOG_DIR/usb_cam.log' 2>&1 & echo \$! > '$PID_DIR/usb_cam.pid'"

echo "[phaseB_start] Starting detector_node..."
cexec "source /opt/ros/jazzy/setup.bash && source /work/demo-cam/ros2_ws/install/setup.bash && \
      nohup ros2 run demo_cam_detect detector_node --ros-args \
        -p model:='$MODEL' \
        -p imgsz:='$IMGSZ' \
        -p conf:='$CONF' \
        -p iou:='$IOU' \
        -p image_topic:='$IMAGE_TOPIC' \
        -p publish_viz:=true \
        -p viz_topic:='$VIZ_TOPIC' \
      > '$LOG_DIR/detector.log' 2>&1 & echo \$! > '$PID_DIR/detector.pid'"

echo "[phaseB_start] Starting mjpeg_bridge..."
cexec "source /opt/ros/jazzy/setup.bash && source /work/demo-cam/ros2_ws/install/setup.bash && \
      nohup ros2 run demo_cam_detect mjpeg_bridge --ros-args \
        -p image_topic:='$VIZ_TOPIC' \
        -p port:='$MJPEG_PORT' \
        -p jpeg_quality:='$JPEG_QUALITY' \
      > '$LOG_DIR/mjpeg_bridge.log' 2>&1 & echo \$! > '$PID_DIR/mjpeg_bridge.pid'"

echo "[phaseB_start] Done."
echo "  - Logs:   $ROOT_DIR/outputs/phaseB/logs/"
echo "  - PIDs:   $ROOT_DIR/outputs/phaseB/pids/"
echo "  - MJPEG:  http://<PI_IP>:${MJPEG_PORT}/stream.mjpg"
echo "Tip: run scripts/phaseB_healthcheck.sh"
