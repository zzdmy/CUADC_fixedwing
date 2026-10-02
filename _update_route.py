# -*- coding: utf-8 -*-
"""将 config.yaml 的航线表更新为 2026-10-07 航点文件换算结果。
- 替换 "# 航线表" 注释块 ~ "# --- RTK" 之间的全部内容
- landing_start_seq: 9 -> 10（新航线投弹后降落段起始 = 任务 seq10）
用法: python _update_route.py <config.yaml>
"""
import sys, io

NEW_BLOCK = """  # 航线表：每条 = 一个航点；bearing_offset 相对起飞航向、distance 相对 HOME、alt 高度
  # 由 2026-10-07 地面站航点文件(10.7航点.waypoints)换算；任务 seq = route下标+2
  # 17 条 = 7 巡航(20m，第7条=seq8=盘旋点/巡航末点) + 1 目标接近点(20m) + 8 降高(20→12m) + 1 着陆(最后一条自动 NAV_LAND)
  route:
    - { bearing_offset_deg: 0.000,    distance_m: 110.25, alt_m: 20 }   # seq2  巡航1（基准方向 = 起飞正前方）
    - { bearing_offset_deg: -3.347,   distance_m: 269.27, alt_m: 20 }   # seq3  巡航2
    - { bearing_offset_deg: 10.841,   distance_m: 261.00, alt_m: 20 }   # seq4  巡航3
    - { bearing_offset_deg: 17.792,   distance_m: 223.60, alt_m: 20 }   # seq5  巡航4
    - { bearing_offset_deg: 11.430,   distance_m: 178.15, alt_m: 20 }   # seq6  巡航5
    - { bearing_offset_deg: -0.400,   distance_m: 192.83, alt_m: 20 }   # seq7  巡航6
    - { bearing_offset_deg: -1.167,   distance_m: 210.49, alt_m: 20 }   # seq8  巡航7 = 盘旋点/巡航末点 ★（切 GUIDED）
    - { bearing_offset_deg: -1.271,   distance_m: 217.86, alt_m: 20 }   # seq9  目标接近点（正常流程跳过）
    - { bearing_offset_deg: -1.263,   distance_m: 225.90, alt_m: 20 }   # seq10 降高1（投弹后从这条跳转）
    - { bearing_offset_deg: -1.004,   distance_m: 260.84, alt_m: 20 }   # seq11 降高2
    - { bearing_offset_deg: 1.213,    distance_m: 321.39, alt_m: 20 }   # seq12 降高3
    - { bearing_offset_deg: 12.349,   distance_m: 314.26, alt_m: 20 }   # seq13 降高4
    - { bearing_offset_deg: 6.960,    distance_m: 272.76, alt_m: 18 }   # seq14 降高5
    - { bearing_offset_deg: 3.111,    distance_m: 230.54, alt_m: 17 }   # seq15 降高6
    - { bearing_offset_deg: 3.235,    distance_m: 179.09, alt_m: 15 }   # seq16 降高7
    - { bearing_offset_deg: 6.974,    distance_m: 104.26, alt_m: 12 }   # seq17 降高8
    - { bearing_offset_deg: 68.947,   distance_m: 19.10,  alt_m: 0 }    # seq18 着陆（自动 NAV_LAND，落点在起飞点旁 19m）

"""

def patch(path):
    with io.open(path, encoding="utf-8") as f:
        text = f.read()
    i1 = text.find("  # 航线表")
    i2 = text.find("# --- RTK")
    if i1 < 0 or i2 < 0 or i2 <= i1:
        print("FAIL: 标记未找到", path)
        return False
    text = text[:i1] + NEW_BLOCK + text[i2:]
    # 降落段起始 seq：9 -> 10（旧注释一并更新）
    old = "landing_start_seq: 9       # 投弹后跳转的降落段起始 MISSION 序号"
    new = "landing_start_seq: 10      # 投弹后跳转的降落段起始 MISSION 序号（新航线=seq10 第一个降高点）"
    if old in text:
        text = text.replace(old, new)
    elif "landing_start_seq: 9" in text:
        import re
        text = re.sub(r"landing_start_seq: 9(\s*#[^\n]*)?",
                      "landing_start_seq: 10      # 投弹后跳转的降落段起始 MISSION 序号（新航线=seq10 第一个降高点）",
                      text, count=1)
    else:
        print("WARN: landing_start_seq 行未按预期匹配，请人工检查", path)
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("OK:", path)
    return True

if __name__ == "__main__":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    for p in sys.argv[1:]:
        patch(p)
