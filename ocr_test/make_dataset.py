# 生成合成靶标数据集：模拟无人机俯拍场景下的真实退化。
#
# 每个样本是一张带编号的地面靶标俯视图，施加以下一种或多种退化：
#   旋转 / 运动模糊 / 传感器噪声 / 透视倾斜 / 远景缩小 / 亮度变化
#
# 输出: dataset/images/<编号>_<退化名>.png  以及 dataset/labels.txt
import math
import os
import random

from PIL import Image, ImageDraw, ImageFilter, ImageFont

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dataset")
IMG_DIR = os.path.join(OUT_DIR, "images")

NUMBERS = ["7", "42", "138", "2", "95", "307", "41", "8", "60", "124"]

# (退化名, 旋转角范围, 模糊半径, 噪声强度, 透视强度, 缩放)
CASES = [
    ("clean",      (0, 0),      0.0, 0,   0.00, 1.00),
    ("rot5",       (-5, 5),     0.0, 0,   0.00, 1.00),
    ("rot15",      (-15, 15),   0.0, 0,   0.00, 1.00),
    ("rot30",      (-30, 30),   0.0, 0,   0.00, 1.00),
    ("blur1",      (0, 0),      1.2, 0,   0.00, 1.00),
    ("blur2",      (0, 0),      2.2, 0,   0.00, 1.00),
    ("noise",      (0, 0),      0.0, 28,  0.00, 1.00),
    ("persp",      (0, 0),      0.0, 0,   0.08, 1.00),
    ("persp_rot",  (-15, 15),   0.0, 0,   0.06, 1.00),
    ("small50",    (0, 0),      0.0, 0,   0.00, 0.50),
    ("small35",    (0, 0),      0.0, 0,   0.00, 0.35),
    ("small25",    (0, 0),      0.8, 0,   0.00, 0.25),
    ("farbig",     (-10, 10),   1.5, 20,  0.05, 0.60),
]


def find_font(size):
    """找一个支持数字的字体。"""
    cands = [
        r"C:\Windows\Fonts\arialbd.ttf",
        r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\calibrib.ttf",
        r"C:\Windows\Fonts\seguisb.ttf",
    ]
    for c in cands:
        if os.path.exists(c):
            try:
                return ImageFont.truetype(c, size)
            except Exception:
                pass
    return ImageFont.load_default()


def make_target(number, seed):
    """生成一张俯拍的靶标图：深色地面上一个白色圆形靶，中心印黑色编号。"""
    rng = random.Random(seed)
    W, H = 900, 700
    img = Image.new("RGB", (W, H), (58, 66, 52))
    d = ImageDraw.Draw(img)

    # 地面纹理：随机深色斑块
    for _ in range(700):
        x = rng.randint(0, W)
        y = rng.randint(0, H)
        r = rng.randint(4, 26)
        v = rng.randint(40, 78)
        d.ellipse([x - r, y - r, x + r, y + r], fill=(v, v + rng.randint(0, 12), v - 6))

    # 靶标：白色圆盘 + 红色外环
    cx, cy = W // 2, H // 2
    R = 175
    d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=(238, 236, 230))
    d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=(170, 40, 40), width=9)

    # 中心编号
    font = find_font(int(R * 0.95) if len(number) <= 2 else int(R * 0.72))
    bbox = d.textbbox((0, 0), number, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    d.text((cx - tw / 2 - bbox[0], cy - th / 2 - bbox[1]), number,
           fill=(18, 18, 18), font=font)

    return img


def solve_homography(src, dst):
    """解 8 参数投影变换，把 src 四点映射到 dst 四点。返回 8 个系数。"""
    A = []
    B = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        B.append(u)
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y])
        B.append(v)
    n = 8
    M = [row[:] + [B[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            return None
        pv = M[c][c]
        M[c] = [v / pv for v in M[c]]
        for r in range(n):
            if r != c and abs(M[r][c]) > 1e-15:
                f = M[r][c]
                M[r] = [a - f * b for a, b in zip(M[r], M[c])]
    return [M[i][n] for i in range(n)]


def apply_perspective(img, strength, rng):
    W, H = img.size
    dx = W * strength
    dy = H * strength
    dst = [(0, 0), (W, 0), (W, H), (0, H)]
    src = [
        (rng.uniform(0, dx), rng.uniform(0, dy)),
        (W - rng.uniform(0, dx), rng.uniform(0, dy)),
        (W - rng.uniform(0, dx), H - rng.uniform(0, dy)),
        (rng.uniform(0, dx), H - rng.uniform(0, dy)),
    ]
    # Image.transform 需要的是"目标像素 -> 源像素"的逆映射
    coef = solve_homography(dst, src)
    if coef is None:
        return img
    return img.transform((W, H), Image.PERSPECTIVE, coef, Image.BICUBIC,
                         fillcolor=(58, 66, 52))


def degrade(img, name, rot, blur, noise, persp, scale, seed):
    rng = random.Random(seed * 7919 + hash(name) % 100000)

    if rot != (0, 0):
        ang = rng.uniform(rot[0], rot[1])
        img = img.rotate(ang, resample=Image.BICUBIC, expand=True,
                         fillcolor=(58, 66, 52))

    if persp > 0:
        img = apply_perspective(img, persp, rng)

    if scale < 1.0:
        w, h = img.size
        img = img.resize((max(32, int(w * scale)), max(32, int(h * scale))),
                         Image.BILINEAR)

    if blur > 0:
        img = img.filter(ImageFilter.GaussianBlur(blur))

    if noise > 0:
        px = img.load()
        w, h = img.size
        for _ in range(int(w * h * 0.05)):
            x, y = rng.randrange(w), rng.randrange(h)
            r, g, b = px[x, y]
            n = rng.randint(-noise, noise)
            px[x, y] = (max(0, min(255, r + n)),
                        max(0, min(255, g + n)),
                        max(0, min(255, b + n)))

    return img


def main():
    os.makedirs(IMG_DIR, exist_ok=True)
    labels = []
    count = 0

    for ni, number in enumerate(NUMBERS):
        base = make_target(number, seed=ni * 101 + 7)
        for ci, (name, rot, blur, noise, persp, scale) in enumerate(CASES):
            seed = ni * 1000 + ci
            img = degrade(base.copy(), name, rot, blur, noise, persp, scale, seed)
            fn = f"{number}_{name}.png"
            path = os.path.join(IMG_DIR, fn)
            img.save(path)
            labels.append((fn, number))
            count += 1

    with open(os.path.join(OUT_DIR, "labels.txt"), "w", encoding="utf-8") as f:
        for fn, num in labels:
            f.write(f"{fn}\t{num}\n")

    print(f"已生成 {count} 张图片 -> {os.path.abspath(IMG_DIR)}")
    print(f"标注文件 -> {os.path.abspath(os.path.join(OUT_DIR, 'labels.txt'))}")
    print(f"编号集合: {NUMBERS}")
    print(f"退化类型: {[c[0] for c in CASES]}")


if __name__ == "__main__":
    main()
