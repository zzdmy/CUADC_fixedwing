# -*- coding: utf-8 -*-
"""验证曝光控制效果：按真实量程(1~10000)逐档设置并测画面亮度"""
import sys
try:
    import cv2
except ImportError:
    print("NO_CV2")
    sys.exit(2)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("open failed")
    sys.exit(1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 手动

for v in (10000, 3000, 1000, 300, 100, 30, 10, 3, 1):
    cap.set(cv2.CAP_PROP_EXPOSURE, v)
    rb = cap.get(cv2.CAP_PROP_EXPOSURE)
    f = None
    for i in range(30):
        ret, f = cap.read()
    print("set %6d -> readback %8s  mean_brightness=%.3f" % (v, rb, (f.mean() / 255.0) if f is not None else -1))
cap.release()
print("done")
