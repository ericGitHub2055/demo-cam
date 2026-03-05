# Phase A Baseline (60s)

**Model**: `models/yolov8n.onnx`  
**Model input**: `[1, 3, 640, 640]`  
**Camera**: 640x480  |  **imgsz**: 640  
**conf/iou**: 0.25/0.45  
**Duration**: 60s  

## Throughput
- Frames processed: **364**
- Avg FPS (processed): **6.07**

## Timing (ms)
| Stage | mean | p50 | p90 | p95 | p99 |
|---|---:|---:|---:|---:|---:|
| preprocess | 3.60 | 3.55 | 3.93 | 3.97 | 4.02 |
| infer      | 156.54 | 156.55 | 157.48 | 157.76 | 158.64 |
| post       | 2.26 | 2.25 | 2.29 | 2.30 | 2.34 |
| total      | 162.40 | 162.33 | 163.37 | 163.74 | 165.34 |

## Notes
- `infer` dominates total latency for CPU-only ORT on Pi.
- CSV with per-frame/per-second samples: `benchmarks/phaseA_640_60s.csv`

