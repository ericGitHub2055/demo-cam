# Phase A Baseline (60s)

**Model**: `models/yolov8n_416.onnx`  
**Model input**: `[1, 3, 416, 416]`  
**Camera**: 640x480  |  **imgsz**: 416  
**conf/iou**: 0.25/0.45  
**Duration**: 60s  

## Throughput
- Frames processed: **909**
- Avg FPS (processed): **15.15**

## Timing (ms)
| Stage | mean | p50 | p90 | p95 | p99 |
|---|---:|---:|---:|---:|---:|
| preprocess | 1.85 | 1.67 | 2.43 | 2.58 | 3.15 |
| infer      | 61.16 | 60.61 | 62.75 | 63.50 | 64.61 |
| post       | 1.50 | 1.51 | 1.56 | 1.57 | 1.60 |
| total      | 64.51 | 63.86 | 66.49 | 67.03 | 69.29 |

## Notes
- `infer` dominates total latency for CPU-only ORT on Pi.
- CSV with per-frame/per-second samples: `benchmarks/phaseA_416_60s.csv`

