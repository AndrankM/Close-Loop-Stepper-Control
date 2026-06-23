#!/usr/bin/env bash
# Downloads the YOLOv4-tiny object-detection model (COCO 80 classes) used by the
# camera object-detection pipeline. Runs on the Pi. Models land in
# ~/led_app/models/ (git-ignored) and are loaded by cv2.dnn.
set -e
mkdir -p ~/led_app/models
cd ~/led_app/models

CFG_URL="https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg"
WEIGHTS_URL="https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov4-tiny.weights"
NAMES_URL="https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names"

echo "-- yolov4-tiny.cfg --"
curl -L --fail --max-time 60 -o yolov4-tiny.cfg "$CFG_URL" || echo "cfg FAILED"
echo "-- yolov4-tiny.weights --"
curl -L --fail --max-time 180 -o yolov4-tiny.weights "$WEIGHTS_URL" || echo "weights FAILED"
echo "-- coco.names --"
curl -L --fail --max-time 60 -o coco.names "$NAMES_URL" || echo "names FAILED"

echo "=== sizes ==="
ls -l ~/led_app/models
echo "=== verify load ==="
python3 - <<'PY'
import os, cv2
base = os.path.expanduser("~/led_app/models")
cfg = os.path.join(base, "yolov4-tiny.cfg")
wts = os.path.join(base, "yolov4-tiny.weights")
names = os.path.join(base, "coco.names")
try:
    net = cv2.dnn.readNetFromDarknet(cfg, wts)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    model = cv2.dnn.DetectionModel(net)
    model.setInputParams(size=(320, 320), scale=1/255.0, swapRB=True)
    with open(names) as f:
        cls = [l.strip() for l in f if l.strip()]
    print("YOLOv4-tiny loaded OK; classes:", len(cls), "weights:", os.path.getsize(wts))
except Exception as e:
    print("YOLO load ERROR:", e)
PY
echo "=== DONE ==="
