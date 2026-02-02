#!/usr/bin/env bash
set -euo pipefail

# Stream H.264 over RTP/UDP and record MP4 simultaneously.
# Default camera: Logitech C920S on /dev/video0 (MJPG capture).
#
# Usage:
#   ./scripts/stream_and_record_h264.sh --host 10.0.0.173 --port 5000 \
#     --res 1280x720 --fps 30 --bitrate 3000 --preset ultrafast --out out.mp4
#
# Stop with Ctrl+C to finalize MP4 correctly (gst-launch -e).

HOST="10.0.0.173"
PORT="5000"
DEV="/dev/video0"
RES="1280x720"
FPS="30"
BITRATE="3000"        # kbps
OUT="out_720p30.mp4"
PT="96"
PRESET="veryfast"     # x264 speed preset

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) HOST="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --dev) DEV="$2"; shift 2;;
    --res) RES="$2"; shift 2;;
    --fps) FPS="$2"; shift 2;;
    --bitrate) BITRATE="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --pt) PT="$2"; shift 2;;
    --preset) PRESET="$2"; shift 2;;
    -h|--help)
      sed -n '1,120p' "$0"
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 1;;
  esac
done

WIDTH="${RES%x*}"
HEIGHT="${RES#*x}"

echo "[INFO] Streaming to ${HOST}:${PORT} (RTP PT=${PT}), recording to ${OUT}"
echo "[INFO] Device=${DEV}, ${WIDTH}x${HEIGHT}@${FPS}, bitrate=${BITRATE} kbps, preset=${PRESET}"

gst-launch-1.0 -e -v \
  v4l2src device="${DEV}" do-timestamp=true ! \
  image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
  jpegdec ! videoconvert ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=${PRESET} bitrate=${BITRATE} key-int-max=$((FPS*2)) ! \
  h264parse config-interval=-1 ! \
  tee name=t \
    t. ! queue ! rtph264pay config-interval=1 pt=${PT} ! \
         udpsink host="${HOST}" port=${PORT} sync=false async=false \
    t. ! queue ! mp4mux faststart=true ! filesink location="${OUT}"
