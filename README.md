# demo-cam

A minimal, reproducible **camera bring-up + streaming + recording** demo on **Raspberry Pi 5** using a **UVC USB webcam** (tested with **Logitech C920S**).

This repo focuses on the fastest path to validate an end-to-end camera pipeline:

- Capture (**MJPEG** from UVC webcam)
- Decode + encode (**H.264** via `x264enc`)
- **Split** into:
  - **RTP/UDP streaming** to a host machine (Windows/macOS)
  - **MP4 recording** on the Raspberry Pi
- (Optional) **ROS 2 bring-up** in Docker:
  - `usb_cam` publishes `/camera/image_raw` + `/camera/camera_info`
  - `apriltag_ros` publishes `/camera/detections`
  - `demo_cam_ros/image_qos_relay` fixes QoS mismatch (RELIABLE → BEST_EFFORT) for vision consumers

---

## What you get

### GStreamer (stream + record)

- `scripts/install_gstreamer.sh`  
  Installs required GStreamer packages and verifies key elements.

- `scripts/stream_and_record_h264.sh`  
  One command to **stream + record simultaneously** (RTP/UDP + MP4). Supports `--preset`, `--gop`, etc.

- `scripts/benchmark_presets.sh`  
  Runs the pipeline with multiple x264 presets, **samples CPU usage**, and extracts video stats via **ffprobe**.  
  Writes results to:
  - `benchmarks/results.csv`
  - `benchmarks/results.md`

- `hostmachine/h264.sdp`  
  An SDP template for **VLC preview** on the host machine (Windows/macOS).

### ROS 2 (AprilTag detection) in Docker

- `docker/compose.yaml` + `docker/Dockerfile`  
  A Dockerized ROS 2 Jazzy environment on Raspberry Pi.

- `ros2_ws/src/demo_cam_ros/launch/usb_cam_apriltag.launch.py`  
  Launches:
  - `usb_cam` node
  - `demo_cam_ros/image_qos_relay` (republish image topic with BEST_EFFORT QoS)
  - `apriltag_ros/apriltag_node` subscribing to the BEST_EFFORT image topic

- `ros2_ws/src/demo_cam_ros/config/usb_cam.yaml`  
  Parameters for `usb_cam` (device, resolution, fps, pixel_format, etc.)

- `ros2_ws/src/demo_cam_ros/config/apriltag.yaml`  
  Parameters for `apriltag_ros` (tag family, max_hamming, optional pose estimation, etc.)

---

## Hardware / OS

- Raspberry Pi 5
- UVC USB webcam (Logitech C920S recommended)
- Raspberry Pi OS (64-bit recommended)

---

## Networking assumptions (GStreamer streaming)

- Raspberry Pi and the host machine are on the **same LAN**
- You know the host machine IP address (Windows/macOS)

### Find your host machine IP

- **Windows**: `ipconfig` → look for **IPv4 Address**
- **macOS**: `ipconfig getifaddr en0` (Wi-Fi is usually `en0`)

---

## Quick Start (GStreamer)

### Where to run each step

- **Raspberry Pi**: run commands under `./scripts/` (install, stream, record, benchmark)
- **Host machine (Windows/macOS)**: use VLC to open the SDP file for preview

### Raspberry Pi (run in a terminal on the Pi)

```bash
cd ~
git clone https://github.com/ericGitHub2055/demo-cam.git
cd ~/demo-cam

./scripts/install_gstreamer.sh
```

#### Baseline (good quality / higher CPU)

```bash
./scripts/stream_and_record_h264.sh   --host <HOST_MACHINE_IP>   --port 5000   --res 1280x720   --fps 30   --bitrate 3000   --preset veryfast   --gop 60   --out out_veryfast.mp4
```

#### Low CPU option (recommended for robotics bring-up)

```bash
./scripts/stream_and_record_h264.sh   --host <HOST_MACHINE_IP>   --port 5000   --res 1280x720   --fps 30   --bitrate 3000   --preset ultrafast   --gop 60   --out out_ultrafast.mp4
```

Stop with `Ctrl+C` to finalize the MP4 file correctly.

---

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

---

## Host preview (VLC)

### Preview on the host machine (Windows/macOS)

Open the SDP file in VLC:

- `hostmachine/h264.sdp`

If VLC shows no video:

- Make sure UDP port `5000` is allowed by the host firewall.
- Confirm the Pi is streaming to `<HOST_MACHINE_IP>` and port `5000`.
- In `hostmachine/h264.sdp`, set `c=IN IP4 <HOST_MACHINE_IP>` if needed.

### Windows host notes (VLC)

If VLC shows no video (traffic cone):

- Confirm the Pi is streaming to your Windows IP and port (default `5000`).
- Allow inbound UDP `5000` in Windows Firewall, or allow VLC as an app.
- If needed, edit `hostmachine/h264.sdp` and set:
  - `c=IN IP4 <HOST_MACHINE_IP>`

---

## Output files

By default, the script writes the MP4 to the current directory, e.g.:

- `~/demo-cam/out_veryfast.mp4`
- `~/demo-cam/out_ultrafast.mp4`

Verify:

```bash
ls -lh out_*.mp4
```

### Copy the MP4 to your host machine

From Windows (PowerShell):

```powershell
scp hello@<PI_IP>:~/demo-cam/out_veryfast.mp4 .
```

From macOS:

```bash
scp hello@<PI_IP>:~/demo-cam/out_veryfast.mp4 .
```

---

## Benchmark x264 presets

This is the recommended way to compare **CPU vs. output characteristics** across x264 presets.

Run a short benchmark (example: 5 seconds per preset):

```bash
./scripts/benchmark_presets.sh --duration 5 --presets "veryfast ultrafast"
```

Outputs:

- `benchmarks/results.csv`
- `benchmarks/results.md`
- `benchmarks/out_<preset>.mp4` (optional to keep; see git note below)

View results:

```bash
cat benchmarks/results.md
column -s, -t < benchmarks/results.csv | head
```

---

## Measurement methodology

### How video performance was measured (ffprobe)

After stopping the pipeline with `Ctrl+C` (so the MP4 is finalized), collect file size and stream stats:

```bash
# File size
ls -lh out_veryfast.mp4

# Container + stream info (duration / bitrate / fps / codec profile)
ffprobe -hide_banner out_veryfast.mp4

# Optional: print only the first ~40 lines (easier to paste into README/issues)
ffprobe -hide_banner out_veryfast.mp4 | sed -n '1,40p'

# Optional: structured output (JSON)
ffprobe -v error   -select_streams v:0   -show_entries stream=codec_name,profile,width,height,r_frame_rate,avg_frame_rate,bit_rate   -show_entries format=duration,bit_rate,size   -of json out_veryfast.mp4
```

### How CPU was measured

CPU usage was measured on the **actual pipeline process** (`gst-launch-1.0`), not the wrapper shell script.

```bash
pgrep -af gst-launch-1.0
# Use the PID printed above:
for i in {1..5}; do
  top -b -n 1 -p <PID> | sed -n '1,12p'
  echo "----"
  sleep 1
done
```

Notes:

- `%CPU` can exceed `100%` on multi-core systems depending on the tool and thread scheduling.
- The benchmark script samples CPU repeatedly and reports average and peak values.

---

## Performance (measured)

Test setup: Raspberry Pi 5 + Logitech C920S (UVC MJPEG input), software encode via `x264enc`, split to RTP/UDP + MP4.

| Mode | Duration | Output size | Video bitrate | FPS | Codec / Profile | Pi CPU (gst-launch) |
|---|---:|---:|---:|---:|---|---:|
| 720p30 RTP + MP4 (`preset=veryfast`, `bitrate=3000`) | 65.68 s | 24 MB | ~2972 kb/s | ~29.72 fps | H.264 (x264) High | 86.7%–120.0% (avg ~102.7%) |
| 720p30 RTP + MP4 (`preset=ultrafast`, `bitrate=3000`) | 56.09 s | 21 MB | ~2994 kb/s | ~29.86 fps | H.264 (x264) Constrained Baseline | 60.0%–73.3% (avg ~65.3%) |

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

## ROS 2 AprilTag bring-up (Docker, Raspberry Pi)

This section is for quickly validating **camera → ROS 2 image topic → AprilTag detections** on the Pi.

### 0) Prerequisites

- Docker + Docker Compose installed on the Pi
- UVC webcam available at `/dev/video0`

### 1) Start the ROS container (persistent)

Run once (or after reboot):

```bash
cd ~/demo-cam
docker compose -f docker/compose.yaml up -d --remove-orphans
docker compose -f docker/compose.yaml ps
```

Expected: the `ros` service is **Up** (the container typically runs `sleep infinity` so you can `exec` into it).

### 2) Build the ROS workspace (inside container)

```bash
docker compose -f docker/compose.yaml exec -it ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /work/demo-cam/ros2_ws &&
  colcon build --symlink-install
'
```

### 3) Launch usb_cam + qos relay + apriltag

Run in one terminal (it stays in the foreground):

```bash
docker compose -f docker/compose.yaml exec -it ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /work/demo-cam/ros2_ws &&
  source install/setup.bash &&
  ros2 launch demo_cam_ros usb_cam_apriltag.launch.py
'
```

### 4) Confirm topics and rates (in another terminal)

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic list | grep -E "^/camera/(image_raw|image_raw_be|camera_info|detections)$" || true
'
```

Typical topics:

- `/camera/image_raw` (from `usb_cam`, usually RELIABLE)
- `/camera/image_raw_be` (from `image_qos_relay`, BEST_EFFORT)
- `/camera/camera_info`
- `/camera/detections`

Check FPS:

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic hz /camera/image_raw -w 50
'
```

### 5) Confirm AprilTag detections

> Detections depend on lighting, focus, motion, tag size/distance, and correct tag family.

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  timeout 5 ros2 topic echo /camera/detections --once || echo "NO DETECTION in 5s"
'
```

If detections are empty (`detections: []`), try:
- better lighting,
- larger tag / closer distance,
- reduce motion blur,
- tune apriltag parameters (see below).

### 6) Inspect node wiring and parameters

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 node info /camera/apriltag
'
```

Common sanity checks:
- `apriltag` subscribes to **`/camera/image_raw_be`** (BEST_EFFORT)
- `apriltag` publishes **`/camera/detections`**

---

## ROS 2 configs (key points)

### Why we need `image_qos_relay`

Many vision nodes (including AprilTag pipelines in practice) prefer `BEST_EFFORT` for image transport to avoid blocking/backpressure.
However, `usb_cam` often publishes images with `RELIABLE`, and **QoS mismatch can lead to “no image received”** behavior.

`demo_cam_ros/image_qos_relay`:
- subscribes to `/camera/image_raw` with a compatible QoS,
- republishes to `/camera/image_raw_be` with `BEST_EFFORT`,
- AprilTag subscribes to `/camera/image_raw_be`.

You can verify QoS with:

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic info /camera/image_raw -v | sed -n "1,120p"
'
```

and

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic info /camera/image_raw_be -v | sed -n "1,120p"
'
```

### Camera calibration warning (pose estimation)

If the camera is not calibrated, `apriltag_ros` can warn about pose estimation.

To **disable pose estimation** (and remove the warning), set:

- `pose_estimation_method: ""` (empty string)

in `ros2_ws/src/demo_cam_ros/config/apriltag.yaml`.

---

## Troubleshooting (ROS 2 Docker)

### 1) `service "ros" is not running`

`docker compose exec` only works if the service is up.

Fix:

```bash
docker compose -f docker/compose.yaml up -d --remove-orphans
docker compose -f docker/compose.yaml ps
```

If `ps` shows nothing, your `compose.yaml` may not keep the container alive.
A common pattern is to run `sleep infinity` as the service command.

### 2) `Couldn't parse params file ... Cannot have a value before ros__parameters`

ROS 2 parameter YAML must be structured correctly. A typical pattern is:

```yaml
/**:
  ros__parameters:
    some_param: 123
```

If you need extra sections (e.g., `qos_overrides`), they **usually must live under `ros__parameters`** for that node.
(And note: not every node implements reading `qos_overrides`—hence the relay node.)

### 3) `Unsupported tag family: tag36h11`

This indicates the installed `apriltag_ros` build does not include the requested tag family at runtime.

What to do:
- Confirm what family is configured:

```bash
docker compose -f docker/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 param get /camera/apriltag family
'
```

- If you need `tag36h11` but the binary rejects it, the practical fix is to
  **use a different supported family** (matching the printed tags you use),
  or rebuild/install a version of `apriltag_ros`/apriltag that includes that family.

### 4) `unknown control 'white_balance_temperature_auto'` (usb_cam)

Some UVC controls are camera-model / driver dependent. This warning is usually harmless.
If you want clean logs, remove or adjust those control settings in `usb_cam` parameters (if present in your node defaults).

### 5) `Ctrl+C` causes Python relay stack trace

A `KeyboardInterrupt` trace in a Python ROS node after Ctrl+C is not a functional error; it just means the process received SIGINT while spinning.
The relay node in this repo is written to handle shutdown cleanly, but depending on timing you may still see an interrupt stack trace.

---

## Notes

- This repo intentionally prioritizes **reproducibility** and **fast validation** over maximum performance.
- The pipeline uses software encoding (`x264enc`) for portability. If your platform provides a hardware H.264 encoder element, you can swap it in later.

---

## Git note (recommended)

Consider ignoring large benchmark outputs:

- `benchmarks/out_*.mp4`
- `benchmarks/cpu_*.txt`

Keep:

- `benchmarks/results.csv`
- `benchmarks/results.md`
