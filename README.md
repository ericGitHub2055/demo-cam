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

## Quick Start

### 1) Clone the repo (on Raspberry Pi)
```bash
cd ~
git clone https://github.com/ericGitHub2055/demo-cam.git
cd demo-cam
