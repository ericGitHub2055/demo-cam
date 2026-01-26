# demo-cam
Reproducible camera pipeline demo on Raspberry Pi 5 + Logitech C920S (GStreamer → ROS2 → perception/behavior).

## Quick Start (Pi)

On Raspberry Pi 5 (Raspberry Pi OS 64-bit):

```bash
# 1) Base setup
./scripts/install_base.sh

# 2) Install GStreamer
./scripts/install_gstreamer.sh

# 3) Probe camera (C920S)
./scripts/camera_probe.sh

# 4) Record 10s MP4
./scripts/gst_record_mp4.sh /dev/video0 out.mp4
