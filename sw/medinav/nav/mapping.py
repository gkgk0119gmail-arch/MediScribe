"""
점유격자 매핑 (occupancy grid mapping, 포즈 알고).

라이다 스캔을 로그오즈(log-odds) 격자에 누적:
  · 광선이 지나간 셀 = 빈칸(free)  → log-odds ↓
  · 광선 끝점 셀     = 벽(occupied) → log-odds ↑
여러 스캔이 쌓이며 맵이 또렷해진다.

포즈는 밖에서 받는다 — 매핑 중에는 라이다 정합(scan_match)과 그래프 SLAM(graph_slam)이 준다.
"""
from __future__ import annotations
import numpy as np


class OccupancyGridMap:
    def __init__(self, size_m=(12.0, 12.0), res=0.05,
                 l_free=-0.4, l_occ=0.85, l_clamp=6.0):
        self.res = float(res)
        self.W = int(round(size_m[0] / res))
        self.H = int(round(size_m[1] / res))
        self.log = np.zeros((self.H, self.W), dtype=np.float32)
        self.l_free, self.l_occ, self.l_clamp = l_free, l_occ, l_clamp

    def integrate(self, pose, angles, ranges, max_range):
        x, y, th = pose
        angles = np.asarray(angles)
        ranges = np.asarray(ranges)
        ca = np.cos(th + angles)
        sa = np.sin(th + angles)
        hit = ranges < max_range - 1e-3
        ends = np.where(hit, ranges, max_range)

        # 1) 빈칸: 각 광선을 따라 끝점 직전까지 샘플
        for i in range(len(angles)):
            stop = max(ends[i] - self.res, 0.0)
            if stop <= 0:
                continue
            ds = np.arange(0.0, stop, self.res)
            cols = ((x + ca[i] * ds) / self.res).astype(int)
            rows = ((y + sa[i] * ds) / self.res).astype(int)
            m = (rows >= 0) & (rows < self.H) & (cols >= 0) & (cols < self.W)
            self.log[rows[m], cols[m]] += self.l_free

        # 2) 벽: 광선 끝점(맞은 경우만)
        hx = x + ca * ranges
        hy = y + sa * ranges
        hc = (hx / self.res).astype(int)
        hr = (hy / self.res).astype(int)
        for i in np.nonzero(hit)[0]:
            if 0 <= hr[i] < self.H and 0 <= hc[i] < self.W:
                self.log[hr[i], hc[i]] += self.l_occ

        np.clip(self.log, -self.l_clamp, self.l_clamp, out=self.log)

    def prob(self):
        """점유 확률 0~1 (1/(1+e^-log))."""
        return 1.0 / (1.0 + np.exp(-self.log))

    # ---------- 저장 / 불러오기 (맵을 시각화·주행에 재사용) ----------
    def save(self, path):
        np.savez(path, log=self.log, res=np.float32(self.res))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        log = d["log"]
        res = float(d["res"])
        H, W = log.shape
        m = cls(size_m=(W * res, H * res), res=res)
        m.log = log.astype(np.float32)
        return m
