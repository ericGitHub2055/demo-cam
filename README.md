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

# Baseline (good quality / higher CPU)
./scripts/stream_and_record_h264.sh \
  --host <HOST_MACHINE_IP> \
  --port 5000 \
  --res 1280x720 \
  --fps 30 \
  --bitrate 3000 \
  --preset veryfast \
  --gop 60 \
  --out out_veryfast.mp4

# Low CPU option (recommended for robotics bring-up)
./scripts/stream_and_record_h264.sh \
  --host <HOST_MACHINE_IP> \
  --port 5000 \
  --res 1280x720 \
  --fps 30 \
  --bitrate 3000 \
  --preset ultrafast \
  --gop 60 \
  --out out_ultrafast.mp4

```

Stop with `Ctrl+C` to finalize the MP4 file correctly.

## Script options

`./scripts/stream_and_record_h264.sh` supports:

- `--host <ip>`: host machine IP (VLC receiver)
- `--port <port>`: UDP port for RTP
- `--dev <device>`: V4L2 device (e.g. `/dev/video0`)
- `--res <WxH>`: resolution (e.g. `1280x720`)
- `--fps <n>`: frame rate
- `--bitrate <kbps>`: H.264 bitrate in kbps
- `--preset <name>`: x264 speed preset (`ultrafast`, `superfast`, `veryfast`, `faster`, `fast`, `medium`, ...)
- `--gop <n>`: GOP size (`key-int-max`)
- `--out <file>`: output MP4 filename
- `--pt <n>`: RTP payload type (default `96`)

Show help:
```bash
./scripts/stream_and_record_h264.sh --help
```

### Preview on the host machine (VLC)

Open the SDP file in VLC:
- `hostmachine/h264.sdp`

If VLC shows no video:
- Make sure UDP port `5000` is allowed by the host firewall.
- Confirm the script is streaming to `<HOST_MACHINE_IP>` and port `5000`.
- In `hostmachine/h264.sdp`, set `c=IN IP4 <HOST_MACHINE_IP>` if needed.

### Windows host notes (VLC)

If VLC shows no video (traffic cone):
- Confirm the Pi is streaming to your Windows IP and port (default `5000`).
- Allow inbound UDP `5000` in Windows Firewall, or allow VLC as an app.
- If needed, edit `hostmachine/h264.sdp` and set:
  - `c=IN IP4 <HOST_MACHINE_IP>`

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

## Performance (measured)

Test setup: Raspberry Pi 5 + Logitech C920S (UVC MJPEG input), software encode via `x264enc`, split to RTP/UDP + MP4.

| Mode | Duration | Output size | Video bitrate | FPS | Codec / Profile | Pi CPU (gst-launch) |
|---|---:|---:|---:|---:|---|---:|
| 720p30 RTP + MP4 (`preset=veryfast`, `bitrate=3000`) | 65.68 s | 24 MB | ~2972 kb/s | ~29.72 fps | H.264 (x264) High | 86.7%–120.0% (avg ~102.7%) |
| 720p30 RTP + MP4 (`preset=ultrafast`, `bitrate=3000`) | 56.09 s | 21 MB | ~2994 kb/s | ~29.86 fps | H.264 (x264) Constrained Baseline | 60.0%–73.3% (avg ~65.3%) |

### How video performance was measured (ffprobe)

After stopping the pipeline with `Ctrl+C` (so the MP4 is finalized), collect file size and stream stats:

```bash
# File size
ls -lh out_720p30.mp4

# Container + stream info (duration / bitrate / fps / codec profile)
ffprobe -hide_banner out_720p30.mp4

# Optional: print only the first ~40 lines (easier to paste into README/issues)
ffprobe -hide_banner out_720p30.mp4 | sed -n '1,40p'

# Optional: structured output (JSON)
ffprobe -v error \
  -select_streams v:0 \
  -show_entries stream=codec_name,profile,width,height,r_frame_rate,avg_frame_rate,bit_rate \
  -show_entries format=duration,bit_rate,size \
  -of json out_720p30.mp4
```

### How CPU was measured
```bash
pgrep -af gst-launch-1.0
# Use the PID printed above:
for i in {1..5}; do top -b -n 1 -p <PID> | sed -n '1,12p'; echo "----"; sleep 1; done

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

