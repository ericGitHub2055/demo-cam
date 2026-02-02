#!/usr/bin/env bash
set -euo pipefail

# Benchmark x264 presets for the stream+record pipeline.
# It runs scripts/stream_and_record_h264.sh with different presets,
# samples CPU on the underlying gst-launch process, and extracts video stats via ffprobe.
#
# Outputs:
#   benchmarks/results.csv
#   benchmarks/results.md
#
# Notes:
# - MP4 produced by mp4mux is typically 0B until the pipeline receives SIGINT/EOS and finalizes.
# - We must send SIGINT to the *gst-launch-1.0* PID (not only the wrapper script PID).

HOST="10.0.0.173"
PORT="5000"
DEV="/dev/video0"
RES="1280x720"
FPS="30"
BITRATE="3000"          # kbps
DURATION="20"           # seconds per preset
SAMPLE_INTERVAL="1"     # seconds
PRESETS_STR="veryfast ultrafast"
PT="96"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PIPE_SCRIPT="${ROOT_DIR}/scripts/stream_and_record_h264.sh"
OUT_DIR="${ROOT_DIR}/benchmarks"
CSV="${OUT_DIR}/results.csv"
MD="${OUT_DIR}/results.md"

usage() {
  cat << EOF
Usage:
  ./scripts/benchmark_presets.sh [options]

Options:
  --host <ip>           Host machine IP (default: ${HOST})
  --port <port>         UDP port (default: ${PORT})
  --dev <device>        V4L2 device (default: ${DEV})
  --res <WxH>           Resolution (default: ${RES})
  --fps <n>             FPS (default: ${FPS})
  --bitrate <kbps>      x264 bitrate (default: ${BITRATE})
  --duration <sec>      seconds per preset (default: ${DURATION})
  --interval <sec>      CPU sample interval seconds (default: ${SAMPLE_INTERVAL})
  --presets "<p1 p2>"   presets list (default: "${PRESETS_STR}")
  --pt <n>              RTP payload type (default: ${PT})
  -h, --help            show help

Example:
  ./scripts/benchmark_presets.sh --duration 20 --presets "veryfast ultrafast"
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
    --duration) DURATION="$2"; shift 2;;
    --interval) SAMPLE_INTERVAL="$2"; shift 2;;
    --presets) PRESETS_STR="$2"; shift 2;;
    --pt) PT="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "[ERR] Unknown arg: $1" >&2; usage; exit 1;;
  esac
done

# Checks
if [[ ! -x "${PIPE_SCRIPT}" ]]; then
  echo "[ERR] Pipeline script not found or not executable: ${PIPE_SCRIPT}" >&2
  echo "[HINT] Ensure scripts/stream_and_record_h264.sh exists and is chmod +x." >&2
  exit 1
fi

if ! command -v ffprobe >/dev/null 2>&1; then
  echo "[ERR] ffprobe not found. Install ffmpeg: sudo apt-get update && sudo apt-get install -y ffmpeg" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

echo "preset,res,fps,bitrate_kbps,duration_target_s,cpu_avg_pct,cpu_peak_pct,file_mb,codec,profile,avg_fps,measured_bitrate_kbps" > "${CSV}"
cat > "${MD}" << 'EOF'
| preset | res | fps | bitrate (kbps) | duration (s) | CPU avg % | CPU peak % | file (MB) | codec/profile | avg fps | bitrate kbps |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|
EOF

echo "[INFO] Benchmarking presets: ${PRESETS_STR}"
echo "[INFO] ${RES}@${FPS}, bitrate=${BITRATE} kbps, duration=${DURATION}s, interval=${SAMPLE_INTERVAL}s"
echo

# Find gst-launch PID that corresponds to a specific output file path
find_gst_pid_for_out() {
  local out="$1"
  local pid=""
  for _ in $(seq 1 80); do
    pid="$(pgrep -n -f "gst-launch-1.0 .*filesink location=${out}" || true)"
    if [[ -n "${pid}" ]]; then
      echo "${pid}"
      return 0
    fi
    sleep 0.1
  done
  return 1
}

calc_cpu_stats_from_file() {
  local f="$1"
  if [[ ! -s "${f}" ]]; then
    echo "0.0 0.0"
    return 0
  fi
  awk '
    {sum+=$1; if($1>max)max=$1; n++}
    END{
      if(n==0) printf "0.0 0.0\n";
      else printf "%.1f %.1f\n", sum/n, max;
    }' "${f}"
}

ffprobe_one_line() {
  local file="${1:-}"
  # Output: codec, profile, avg_frame_rate, bit_rate
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_name,profile,avg_frame_rate,bit_rate \
    -of default=noprint_wrappers=1:nokey=0 "${file}"
}

frac_to_fps() {
  local r="$1"
  python3 - << 'PY'
import os
r=os.environ["R"]
if "/" in r:
    a,b=r.split("/",1)
    try:
        print(f"{float(a)/float(b):.2f}")
    except:
        print("0.00")
else:
    try:
        print(f"{float(r):.2f}")
    except:
        print("0.00")
PY
}

for preset in ${PRESETS_STR}; do
  out_mp4="${OUT_DIR}/out_${preset}.mp4"
  cpu_s="${OUT_DIR}/cpu_${preset}.txt"
  rm -f "${out_mp4}" "${cpu_s}"

  echo "[INFO] Running preset=${preset} -> ${out_mp4}"
  echo "  cmd: ${PIPE_SCRIPT} --host ${HOST} --port ${PORT} --dev ${DEV} --res ${RES} --fps ${FPS} --bitrate ${BITRATE} --preset ${preset} --pt ${PT} --out ${out_mp4}"

  # Start wrapper (it runs gst-launch in foreground)
  "${PIPE_SCRIPT}" --host "${HOST}" --port "${PORT}" --dev "${DEV}" --res "${RES}" --fps "${FPS}" \
    --bitrate "${BITRATE}" --preset "${preset}" --pt "${PT}" --out "${out_mp4}" >/dev/null 2>&1 &
  wrapper_pid=$!

  # Find underlying gst-launch pid for this output file
  if ! gst_pid="$(find_gst_pid_for_out "${out_mp4}")"; then
    echo "[ERR] Cannot find gst-launch PID for output: ${out_mp4}" >&2
    kill -INT "${wrapper_pid}" >/dev/null 2>&1 || true
    wait "${wrapper_pid}" >/dev/null 2>&1 || true
    exit 1
  fi

  echo "[INFO] wrapper_pid=${wrapper_pid}, gst_pid=${gst_pid}"

  # CPU sampling loop (on gst-launch PID)
  start_ts="$(date +%s)"
  while true; do
    now_ts="$(date +%s)"
    elapsed=$((now_ts - start_ts))
    (( elapsed >= DURATION )) && break

    cpu="$(ps -p "${gst_pid}" -o %cpu= 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "${cpu}" ]]; then
      echo "${cpu}" >> "${cpu_s}"
    fi
    sleep "${SAMPLE_INTERVAL}"
  done

  # Stop gst-launch cleanly (this is the key!)
  kill -INT "${gst_pid}" >/dev/null 2>&1 || true

  # Also interrupt wrapper in case it doesn't exit
  kill -INT "${wrapper_pid}" >/dev/null 2>&1 || true

  # Wait for wrapper to exit (give it a moment)
  set +e
  wait "${wrapper_pid}" >/dev/null 2>&1
  set -e

  # Give mp4mux time to flush
  sleep 1

  # CPU stats
  read -r cpu_avg cpu_peak < <(calc_cpu_stats_from_file "${cpu_s}")

  if [[ ! -s "${out_mp4}" ]]; then
    echo "[ERR] Output file is empty after SIGINT: ${out_mp4}" >&2
    echo "${preset},${RES},${FPS},${BITRATE},${DURATION},${cpu_avg},${cpu_peak},0,NA,NA,0,0" >> "${CSV}"
    printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |\n" \
      "${preset}" "${RES}" "${FPS}" "${BITRATE}" "${DURATION}" "${cpu_avg}" "${cpu_peak}" "0" "NA/NA" "0" "0" >> "${MD}"
    continue
  fi

  # File MB
  file_mb="$(python3 - <<PY
import os
p="${out_mp4}"
print(f"{os.path.getsize(p)/1024/1024:.1f}")
PY
)"

  # ffprobe stats
  info="$(ffprobe_one_line "${out_mp4}")"
  codec="$(printf "%s\n" "${info}" | awk -F= '$1=="codec_name"{print $2; exit}')"
  profile="$(printf "%s\n" "${info}" | awk -F= '$1=="profile"{print $2; exit}')"
  afr="$(printf "%s\n" "${info}" | awk -F= '$1=="avg_frame_rate"{print $2; exit}')"
  br_bps="$(printf "%s\n" "${info}" | awk -F= '$1=="bit_rate"{print $2; exit}')"

  export R="${afr}"
  avg_fps="$(frac_to_fps "${afr}")"
  measured_bitrate_kbps="$(python3 - <<PY
try:
  print(int(round(float("${br_bps}")/1000.0)))
except:
  print(0)
PY
)"

  echo "${preset},${RES},${FPS},${BITRATE},${DURATION},${cpu_avg},${cpu_peak},${file_mb},${codec},${profile},${avg_fps},${measured_bitrate_kbps}" >> "${CSV}"
  printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s/%s | %s | %s |\n" \
    "${preset}" "${RES}" "${FPS}" "${BITRATE}" "${DURATION}" "${cpu_avg}" "${cpu_peak}" "${file_mb}" "${codec}" "${profile}" "${avg_fps}" "${measured_bitrate_kbps}" >> "${MD}"

  echo "[OK] preset=${preset}: CPU avg=${cpu_avg}%, peak=${cpu_peak}%, file=${file_mb}MB, ${codec}/${profile}, fps=${avg_fps}, br=${measured_bitrate_kbps} kbps"
  echo
done

echo "[DONE] Wrote:"
echo "  ${CSV}"
echo "  ${MD}"
