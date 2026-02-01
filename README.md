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
  One command to **stream + record simultaneously** (RTP/UDP + MP4).

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

- **Windows**: `ipconfig` → look for **IPv4 Address**
- **macOS**: `ipconfig getifaddr en0` (Wi-Fi is usually `en0`)

---

## Quick Start

### Where to run each step

- **Raspberry Pi**: run all commands under `./scripts/` (install, stream, record)
- **Host machine (Windows/macOS)**: use VLC to open the SDP file for preview

### Raspberry Pi (run in a terminal on the Pi)

```bash
cd ~
git clone https://github.com/ericGitHub2055/demo-cam.git
cd ~/demo-cam

./scripts/install_gstreamer.sh

./scripts/stream_and_record_h264.sh \
  --host <HOST_MACHINE_IP> \
  --port 5000 \
  --res 1280x720 \
  --fps 30 \
  --bitrate 3000 \
  --out out_720p30.mp4
```

Stop with `Ctrl+C` to finalize the MP4 file correctly.

### Preview on the host machine (VLC)

Open the SDP file in VLC:
- `hostmachine/h264.sdp`

If VLC shows no video:
- Make sure UDP port `5000` is allowed by the host firewall.
- Confirm the script is streaming to `<HOST_MACHINE_IP>` and port `5000`.
- In `hostmachine/h264.sdp`, set `c=IN IP4 <HOST_MACHINE_IP>` if needed.

### Output files

By default, the script writes the MP4 to the current directory, e.g.:
- `~/demo-cam/out_720p30.mp4`

Verify:
```bash
ls -lh out_*.mp4
```

### Copy the MP4 to your host machine

From Windows (PowerShell):
```powershell
scp hello@<PI_IP>:~/demo-cam/out_720p30.mp4 .
```

From macOS:
```bash
scp hello@<PI_IP>:~/demo-cam/out_720p30.mp4 .
```

---

## Optional: verify the webcam

List devices:
```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

Check formats (example for `/dev/video0`):
```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

For Logitech C920S, **MJPG** is typically required for stable **720p30 / 1080p30**.  
Raw YUYV at higher resolutions may be limited by USB bandwidth.

---

## Notes

- This repo intentionally prioritizes **reproducibility** and **fast validation** over maximum performance.
- The pipeline uses software encoding (`x264enc`) for portability. If your platform provides a hardware H.264 encoder element, you can swap it in later.
