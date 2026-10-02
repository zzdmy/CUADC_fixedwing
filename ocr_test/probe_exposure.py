# -*- coding: utf-8 -*-
"""探测相机曝光真实范围：逐档设置并回读(驱动会把超范围值截断到真实边界)"""
import sys
try:
    import cv2
except ImportError:
    print("NO_CV2")
    sys.exit(2)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("opened:", cap.isOpened())
if not cap.isOpened():
    sys.exit(1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 手动模式

print("--- 探测下限(回读=驱动真实最小) ---")
for v in (-6, -10, -13, -16, -20, -100):
    ok = cap.set(cv2.CAP_PROP_EXPOSURE, v)
    rb = cap.get(cv2.CAP_PROP_EXPOSURE)
    print("set %6d -> ok=%s readback=%s" % (v, ok, rb))

print("--- 探测上限 ---")
for v in (0, 100, 1000):
    cap.set(cv2.CAP_PROP_EXPOSURE, v)
    print("set %6d -> readback=%s" % (v, cap.get(cv2.CAP_PROP_EXPOSURE)))

print("--- 最低曝光下画面亮度(预热后) ---")
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -100)  # 会被截断到真实最低
for i in range(60):
    ret, f = cap.read()
if ret:
    print("mean brightness at min exposure: %.3f" % (f.mean() / 255.0))
cap.release()
print("done")
