# demo-cam
Reproducible camera pipeline demo on Raspberry Pi 5 + Logitech C920S (GStreamer → ROS2 → perception/behavior).

## Quick Start (Raspberry Pi)

Install dependencies:
```bash
./scripts/install_gstreamer.sh

./scripts/stream_and_record_h264.sh --host <HOST_MACHINE_IP> --port 5000 --res 1280x720 --fps 30 --bitrate 3000 --out out_720p30.mp4

