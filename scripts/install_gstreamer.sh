#!/usr/bin/env bash
set -euo pipefail

# Install GStreamer (plus common plugins) and basic utilities on Raspberry Pi OS.
# This enables:
# - MJPEG capture from UVC webcams (e.g., Logitech C920S)
# - H.264 software encoding via x264enc
# - MP4 muxing and RTP/UDP streaming
#
# Usage:
#   ./scripts/install_gstreamer.sh

sudo apt update
sudo apt install -y \
  git \
  v4l-utils \
  ffmpeg \
  tcpdump \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav

echo ""
echo "[OK] Installed GStreamer and plugins."
echo "[INFO] Verifying key elements..."
gst-inspect-1.0 x264enc >/dev/null && echo "[OK] x264enc found" || echo "[ERR] x264enc not found"
gst-inspect-1.0 rtph264pay >/dev/null && echo "[OK] rtph264pay found" || echo "[ERR] rtph264pay not found"
gst-inspect-1.0 mp4mux >/dev/null && echo "[OK] mp4mux found" || echo "[ERR] mp4mux not found"
echo "[DONE]"
