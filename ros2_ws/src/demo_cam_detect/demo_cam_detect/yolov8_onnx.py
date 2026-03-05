#!/usr/bin/env python3
import argparse
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import onnxruntime as ort


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))

    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2

    im_resized = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im_padded = cv2.copyMakeBorder(
        im_resized, top, bottom, left, right,
        cv2.BORDER_CONSTANT, value=color
    )
    return im_padded, r, (left, top)


def nms_boxes(boxes, scores, iou_thres=0.45):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= iou_thres)[0]
        order = order[inds + 1]
    return keep


def postprocess_yolov8(pred, conf_thres=0.25, iou_thres=0.45):
    # Your model output: (1, 84, 8400)
    pred = pred[0].T  # (8400,84)
    boxes = pred[:, :4]       # xywh
    cls_scores = pred[:, 4:]  # (8400,80)

    cls_id = np.argmax(cls_scores, axis=1)
    scores = cls_scores[np.arange(cls_scores.shape[0]), cls_id]

    keep = scores > conf_thres
    boxes = boxes[keep]
    scores = scores[keep]
    cls_id = cls_id[keep]
    if boxes.shape[0] == 0:
        return []

    xyxy = np.zeros_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    results = []
    for c in np.unique(cls_id):
        inds = np.where(cls_id == c)[0]
        b = xyxy[inds]
        s = scores[inds]
        k = nms_boxes(b, s, iou_thres=iou_thres)
        for j in k:
            results.append((b[j], float(s[j]), int(c)))
    return results


