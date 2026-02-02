#!/usr/bin/env bash
set -euo pipefail

# Stream H.264 over RTP/UDP and record MP4 simultaneously.
# Default camera: Logitech C920S on /dev/video0 (MJPG capture).
#
# Usage examples:
#   ./scripts/stream_and_record_h264.sh
#   ./scripts/stream_and_record_h264.sh --host 10.0.0.173 --port 5000 --res 1280x720 --fps 30 --bitrate 3000 --preset veryfast --gop 60 --out out.mp4
#
# Notes:
# - Requires: gstreamer1.0-plugins-{base,good,bad,ugly}, gstreamer1.0-tools
# - Stop with Ctrl+C to finalize MP4 correctly (gst-launch -e).

HOST="10.0.0.173"
PORT="5000"
DEV="/dev/video0"
RES="1280x720"
FPS="30"

BITRATE="3000"     # kbps
PRESET="veryfast"  # x264 speed-preset: ultrafast/superfast/veryfast/faster/fast/medium/...
GOP=""             # key-int-max; empty => default to FPS*2

OUT="out_720p30.mp4"
PT="96"

usage() {
  cat << 'EOF'
Usage:
  ./scripts/stream_and_record_h264.sh [options]

Options:
  --host <ip>         Host machine IP (default: 10.0.0.173)
  --port <port>       UDP port for RTP (default: 5000)
  --dev <device>      V4L2 device (default: /dev/video0)
  --res <WxH>         Resolution (default: 1280x720)
  --fps <n>           Frame rate (default: 30)
  --bitrate <kbps>    H.264 bitrate in kbps (default: 3000)
  --preset <name>     x264 speed-preset (default: veryfast)
  --gop <n>           GOP size / key-int-max (default: FPS*2)
  --out <file>        Output MP4 filename (default: out_720p30.mp4)
  --pt <n>            RTP payload type (default: 96)
  -h, --help          Show this help

Examples:
  ./scripts/stream_and_record_h264.sh
  ./scripts/stream_and_record_h264.sh --host 10.0.0.173 --preset ultrafast --bitrate 2000 --out out_ultrafast.mp4
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) HOST="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --dev) DEV="$2"; shift 2;;
    --res) RES="$2"; shift 2;;
    --fps) FPS="$2"; shift 2;;
    --bitrate) BITRATE="$2"; shift 2;;
    --preset) PRESET="$2"; shift 2;;
    --gop) GOP="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --pt) PT="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 1;;
  esac
done

WIDTH="${RES%x*}"
HEIGHT="${RES#*x}"

if [[ -z "${GOP}" ]]; then
  GOP=$((FPS*2))
fi

echo "[INFO] Streaming to ${HOST}:${PORT} (RTP PT=${PT}), recording to ${OUT}"
echo "[INFO] Device=${DEV}, ${WIDTH}x${HEIGHT}@${FPS}, bitrate=${BITRATE} kbps, preset=${PRESET}, gop=${GOP}"

gst-launch-1.0 -e -v \
  v4l2src device="${DEV}" do-timestamp=true ! \
  image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
  jpegdec ! videoconvert ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=${PRESET} bitrate=${BITRATE} key-int-max=${GOP} ! \
  h264parse config-interval=-1 ! \
  tee name=t \
    t. ! queue ! rtph264pay config-interval=1 pt=${PT} ! \
         udpsink host="${HOST}" port=${PORT} sync=false async=false \
    t. ! queue ! mp4mux faststart=true ! filesink location="${OUT}"
