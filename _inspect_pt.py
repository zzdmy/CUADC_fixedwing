# -*- coding: utf-8 -*-
"""读取 best.pt 模型档案：类别/任务/输入尺寸/训练参数等"""
import torch, json

p = r"C:\Users\33083\Desktop\best.pt"
ckpt = torch.load(p, map_location="cpu", weights_only=False)

print("== ckpt keys ==")
print(list(ckpt.keys()))

m = ckpt.get("model")
print("\n== model ==")
print("type:", type(m).__name__)
names = getattr(m, "names", None)
print("names:", names)
try:
    n = sum(p_.numel() for p_ in m.parameters())
    print("params: %.2fM" % (n / 1e6))
except Exception as e:
    print("params err:", e)

y = getattr(m, "yaml", None)
print("\n== model.yaml ==")
if isinstance(y, dict):
    for k in ("yaml_file", "task", "nc", "scale", "imgsz", "ch",
              "depth_multiple", "width_multiple"):
        if k in y:
            print(f"  {k}: {y[k]}")
    print("  keys:", sorted(y.keys())[:20])
else:
    print(" ", y)

ta = ckpt.get("train_args") or {}
print("\n== train_args ==")
for k in ("model", "data", "epochs", "imgsz", "batch", "optimizer", "lr0",
          "device", "project", "name", "pretrained", "rect", "workers"):
    if k in ta:
        print(f"  {k}: {ta[k]}")

print("\n== 训练元数据 ==")
for k in ("date", "version", "epoch", "best_fitness", "train_metrics",
          "optimizer", "ema"):
    v = ckpt.get(k)
    if k == "train_metrics" and v:
        v = dict(v)
    if v is not None and k != "optimizer" and k != "ema":
        print(f"  {k}:", v)
print("  has_ema:", "ema" in ckpt, " has_optimizer:", "optimizer" in ckpt)
