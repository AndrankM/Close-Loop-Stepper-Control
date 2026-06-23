#!/usr/bin/env bash
set -e
PW='12345'
echo "=== apt update + install python3-opencv ==="
echo "$PW" | sudo -S apt-get update -qq
echo "$PW" | sudo -S DEBIAN_FRONTEND=noninteractive apt-get install -y python3-opencv
echo "=== verify cv2 ==="
python3 - <<'PY'
import cv2, numpy
print("cv2", cv2.__version__, "numpy", numpy.__version__)
print("has FaceDetectorYN:", hasattr(cv2, "FaceDetectorYN"))
PY
echo "=== download models ==="
mkdir -p ~/led_app/models
cd ~/led_app/models
FER_URL="https://media.githubusercontent.com/media/onnx/models/main/validated/vision/body_analysis/emotion_ferplus/model/emotion-ferplus-8.onnx"
YUNET_URL="https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
echo "-- FER+ --"
curl -L --fail --max-time 120 -o emotion-ferplus-8.onnx "$FER_URL" || echo "FER download FAILED"
echo "-- YuNet --"
curl -L --fail --max-time 120 -o face_detection_yunet_2023mar.onnx "$YUNET_URL" || echo "YuNet download FAILED"
echo "=== sizes ==="
ls -l ~/led_app/models
echo "=== verify models load ==="
python3 - <<'PY'
import os, cv2
base = os.path.expanduser("~/led_app/models")
fer = os.path.join(base, "emotion-ferplus-8.onnx")
yun = os.path.join(base, "face_detection_yunet_2023mar.onnx")
try:
    net = cv2.dnn.readNetFromONNX(fer)
    print("FER+ loaded OK, size", os.path.getsize(fer))
except Exception as e:
    print("FER+ load ERROR:", e)
try:
    fd = cv2.FaceDetectorYN.create(yun, "", (320, 320))
    print("YuNet loaded OK, size", os.path.getsize(yun))
except Exception as e:
    print("YuNet load ERROR:", e)
PY
echo "=== DONE ==="
