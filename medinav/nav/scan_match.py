"""
스캔매칭 (likelihood-field, 순수 파이썬/numpy) — SLAM의 'SL'(위치교정).

오도메로 예측한 포즈 주변을 (dx, dy, dθ)로 탐색하며, 라이다 스캔점이
지금까지 만든 맵의 '벽 근처'에 가장 잘 얹히는 포즈를 고른다.
→ 오도메 드리프트를 라이다로 계속 교정 → 맵이 안 꺾임.

방법:
  · likelihood field = 점유격자의 '벽' 셀을 흐릿하게 번지게 한 장(가까울수록 점수↑)
  · 스캔점을 후보 포즈로 월드변환 → 필드값 합 = 정합점수
  · coarse→fine 2단 탐색으로 최적 포즈
"""
from __future__ import annotations
import math
import numpy as np


def _box_blur(a, iters=3):
    """3x3 평균을 iters번 → 근사 가우시안(벽 주변으로 점수 번짐). scipy 불필요."""
    out = a.astype(np.float32)
    for _ in range(iters):
        p = np.pad(out, 1, mode="edge")
        out = (p[:-2, 1:-1] + p[2:, 1:-1] + p[1:-1, :-2] +
               p[1:-1, 2:] + p[1:-1, 1:-1]) / 5.0
    return out


class ScanMatcher:
    def __init__(self, gm, occ_thresh=0.55, blur_iters=3):
        self.gm = gm
        self.res = gm.res
        self.W, self.H = gm.W, gm.H
        self.occ_thresh = occ_thresh
        self.blur_iters = blur_iters
        self.field = np.zeros((gm.H, gm.W), dtype=np.float32)

    def update_field(self):
        """현재 맵에서 likelihood field 재계산 (맵 채워질수록 주기적으로 호출)."""
        occ = (self.gm.prob() > self.occ_thresh).astype(np.float32)
        self.field = _box_blur(occ, self.blur_iters)

    def _score(self, x, y, th, pts):
        c, s = math.cos(th), math.sin(th)
        wx = x + pts[:, 0] * c - pts[:, 1] * s
        wy = y + pts[:, 0] * s + pts[:, 1] * c
        cols = (wx / self.res).astype(np.int32)
        rows = (wy / self.res).astype(np.int32)
        m = (rows >= 0) & (rows < self.H) & (cols >= 0) & (cols < self.W)
        if not m.any():
            return 0.0
        return float(self.field[rows[m], cols[m]].sum())

    def _score_grid(self, xs, ys, th, pts):
        """th 고정, xs×ys 모든 병진후보 점수를 '한 방에'(벡터화). 반환 (X평탄, Y평탄, 점수)."""
        c, s = math.cos(th), math.sin(th)
        rx = pts[:, 0] * c - pts[:, 1] * s          # 회전만(병진 전) (Np,)
        ry = pts[:, 0] * s + pts[:, 1] * c
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        X = X.ravel(); Y = Y.ravel()                # (Nc,) 후보들
        wx = X[:, None] + rx[None, :]               # (Nc, Np)
        wy = Y[:, None] + ry[None, :]
        cols = (wx / self.res).astype(np.int32)
        rows = (wy / self.res).astype(np.int32)
        valid = (rows >= 0) & (rows < self.H) & (cols >= 0) & (cols < self.W)
        np.clip(rows, 0, self.H - 1, out=rows)
        np.clip(cols, 0, self.W - 1, out=cols)
        scores = (self.field[rows, cols] * valid).sum(axis=1)   # (Nc,)
        return X, Y, scores

    def match(self, pose, pts,
              coarse=(0.18, math.radians(11.0), 5),
              fine=(0.02, math.radians(1.5), 3)):
        """
        pose: 오도메 예측 (x, y, th) [맵-월드 m]
        pts:  로봇프레임 스캔점 Nx2 (정합용, 솎아쓰기 권장)
        반환: 보정된 (x, y, th).  x,y 격자는 벡터화, θ만 파이썬 루프(coarse 11·fine 7회).
        """
        best = pose
        base = self._score(pose[0], pose[1], pose[2], pts)
        if base <= 0.0:                     # 맵이 아직 비어 정합할 게 없음 → 예측 그대로
            return pose
        for rad_lin, rad_ang, n in (coarse, fine):
            bx, by, bth = best
            bs = self._score(bx, by, bth, pts)
            xs = np.linspace(bx - rad_lin, bx + rad_lin, 2 * n + 1)
            ys = np.linspace(by - rad_lin, by + rad_lin, 2 * n + 1)
            ths = np.linspace(bth - rad_ang, bth + rad_ang, 2 * n + 1)
            for th in ths:
                X, Y, scores = self._score_grid(xs, ys, th, pts)
                k = int(scores.argmax())
                if scores[k] > bs:
                    bs, best = float(scores[k]), (float(X[k]), float(Y[k]), float(th))
        return best
