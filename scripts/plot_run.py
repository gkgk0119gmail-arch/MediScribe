#!/usr/bin/env python3
"""주행 기록(.log) 을 지도(.npz) 위에 그린다 — 궤적을 상태(전진/후진 회전/탈출/도착)별 색으로.

  python3 scripts/plot_run.py --map out/joymap_<시각>.npz --log out/joymap_<시각>_nav_<시각>.log -o run.png
  python3 scripts/plot_run.py --map out/joymap_<시각>.npz -o map.png          # 지도만

지도 npz: 'log'(점유 로그오즈, 행 = y) + 'res'. 지도 중심이 매핑 시작점(0, 0) 이다.
주행 로그 한 줄: "HH:MM:SS [mode] ... pose (+x,+y, θ°) ..." (좌표는 지도 중심 기준 m).
"""
import argparse, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MODE_STYLE = {   # 상태 → (색, 한글 이름)
    "drive":   ("#1e88e5", "전진"),
    "reverse": ("#e53935", "후진 회전 (목표가 뒤쪽)"),
    "escape":  ("#fb8c00", "탈출 후진 (앞 막힘)"),
    "align":   ("#8e24aa", "방향 맞춤"),
    "arrived": ("#43a047", "도착"),
}
LINE = re.compile(r"^(\d\d:\d\d:\d\d) \[([a-z]+)\].*?pose \(([+-][\d.]+),([+-][\d.]+),\s*([+-]?[\d.]+)°\)")


def load_map(path):
    d = np.load(path)
    lo, res = d["log"].astype(np.float32), float(d["res"])
    p = 1.0 / (1.0 + np.exp(-lo))
    img = np.full(p.shape, 0.80, np.float32)          # 미지 = 회색
    img[p < 0.4] = 1.0                                # 빈 곳 = 흰색
    img[p > 0.6] = 0.05                               # 벽 = 검정
    half = p.shape[0] * res / 2
    known = np.argwhere((p < 0.4) | (p > 0.6))
    (r0, c0), (r1, c1) = known.min(0), known.max(0)
    box = (c0 * res - half, c1 * res - half, r0 * res - half, r1 * res - half)
    return img, half, box


def load_run(path):
    rows = []
    for ln in open(path, encoding="utf-8", errors="ignore"):
        m = LINE.match(ln)
        if m:
            rows.append((m.group(1), m.group(2), float(m.group(3)), float(m.group(4)), float(m.group(5))))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True); ap.add_argument("--log"); ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--title", default=""); ap.add_argument("--margin", type=float, default=1.0)
    a = ap.parse_args()
    import logging; logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    from matplotlib import font_manager as fm               # 한글 글꼴: 설치된 CJK 글꼴 중 있는 것을 쓴다
    names = {f.name for f in fm.fontManager.ttflist}
    plt.rcParams["font.family"] = [n for n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic",
                                               "AppleGothic", "Malgun Gothic") if n in names] + ["DejaVu Sans"]
    img, half, (x0, x1, y0, y1) = load_map(a.map)
    fig, ax = plt.subplots(figsize=(8, 8 * (y1 - y0 + 2 * a.margin) / max(1e-3, x1 - x0 + 2 * a.margin)), dpi=150)
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=[-half, half, -half, half], interpolation="nearest")
    stats = ""
    if a.log:
        rows = load_run(a.log)
        moving = [r for r in rows if r[1] in MODE_STYLE]
        for (t0, m0, xa, ya, _), (t1, m1, xb, yb, _) in zip(rows, rows[1:]):
            if m0 in MODE_STYLE and m0 != "arrived":
                ax.plot([xa, xb], [ya, yb], color=MODE_STYLE[m0][0], lw=2.6, solid_capstyle="round")
        arrivals = [r for a_, r in zip(rows, rows[1:]) if r[1] == "arrived" and a_[1] != "arrived"]
        for r in arrivals:
            ax.plot(r[2], r[3], "o", ms=9, mfc=MODE_STYLE["arrived"][0], mec="white", mew=1.5, zorder=5)
        if rows:
            ax.plot(rows[0][2], rows[0][3], "s", ms=9, mfc="black", mec="white", mew=1.5, zorder=5)
        used = sorted({r[1] for r in moving}, key=list(MODE_STYLE).index)
        handles = [plt.Line2D([], [], color=MODE_STYLE[m][0], lw=3 if m != "arrived" else 0, marker="o" if m == "arrived" else None,
                              ms=8, label=MODE_STYLE[m][1]) for m in used]
        handles.append(plt.Line2D([], [], color="black", lw=0, marker="s", ms=8, label="출발"))
        ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.9)
        n_rev = sum(1 for p_, r in zip(rows, rows[1:]) if r[1] in ("reverse", "escape") and p_[1] not in ("reverse", "escape"))
        stats = f"목표 도착 {len(arrivals)}회 · 후진 기동 {n_rev}회 · {rows[0][0]}–{rows[-1][0]}" if rows else ""
    ax.set_xlim(max(-half, x0 - a.margin), min(half, x1 + a.margin))        # 지도 밖으로는 넓히지 않는다
    ax.set_ylim(max(-half, y0 - a.margin), min(half, y1 + a.margin))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal")
    ax.set_title(a.title + (("\n" + stats) if stats else ""), fontsize=10)
    fig.tight_layout(); fig.savefig(a.out); print("저장:", a.out, "|", stats)


if __name__ == "__main__":
    main()
