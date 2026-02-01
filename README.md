# demo-cam

A minimal, reproducible **camera bring-up + streaming + recording** demo on **Raspberry Pi 5** using a **UVC USB webcam** (tested with **Logitech C920S**).

This repo focuses on the fastest path to validate an end-to-end camera pipeline:
- Capture (MJPEG from UVC webcam)
- Decode + encode (H.264 via `x264enc`)
- **Split** into:
  - **RTP/UDP streaming** to a host machine (Windows/macOS)
  - **MP4 recording** on the Raspberry Pi

---

## What you get

- `scripts/install_gstreamer.sh`  
  Installs required GStreamer packages and verifies key elements.

- `scripts/stream_and_record_h264.sh`  
  One command to **stream + record simultaneously**.

- `hostmachine/h264.sdp`  
  An SDP template for **VLC preview** on the host machine (Windows/macOS).

---

## Hardware / OS

- Raspberry Pi 5
- UVC USB webcam (Logitech C920S recommended)
- Raspberry Pi OS (64-bit recommended)

---

## Networking assumptions

- Raspberry Pi and the host machine are on the **same LAN**
- You know the host machine IP address (Windows/macOS)

### Find your host machine IP
- **Windows**: `ipconfig` → look for IPv4 Address
- **macOS**: `ipconfig getifaddr en0` (Wi-Fi usually `en0`)

---

### Where to run each step

- **Raspberry Pi**: run all commands under `./scripts/` (install, stream, record)
- **Host machine (Windows/macOS)**: use VLC to open the SDP file for preview

### Raspberry Pi (run in a terminal on the Pi)
```bash
cd ~/demo-cam
./scripts/install_gstreamer.sh
./scripts/stream_and_record_h264.sh --host <HOST_MACHINE_IP> --port 5000 --out out_720p30.mp4

