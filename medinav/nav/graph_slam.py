"""
그래프 기반 SLAM (프론트엔드 + 루프탐지) — 외부 SLAM 라이브러리 없이 순수 파이썬/numpy 로 직접 구현.

구성:
  · 키프레임: 일정 거리/각도마다 (포즈 + 스캔) 저장 = 그래프의 노드
  · 오도메 엣지: 연속 키프레임 사이 상대변환(스캔매칭 보정) 제약
  · 루프탐지: 새 키프레임이 과거 키프레임 근처면 그 스캔과 재정합 → 루프 엣지
  · 최적화(pose_graph): 루프 엣지가 누적 드리프트를 전체 궤적에 분산 → 맵 닫힘
  · 맵 재구성: 최적화된 키프레임 포즈로 점유격자 다시 그림

입력: process(odom_pose(x,y,θ), angles[rad,로봇], ranges[m])
"""
from __future__ import annotations
import math
import numpy as np

from .mapping import OccupancyGridMap
from .scan_match import ScanMatcher
from .pose_graph import PoseGraph
from ..core.se2 import rel as _rel, compose as _compose, wrap as _wrap   # SE(2) 유틸 정본


class GraphSLAM:
    def __init__(self, map_m=24.0, res=0.05, max_range=8.0,
                 kf_dist=0.5, kf_ang=math.radians(25),
                 loop_radius=1.2, loop_min_gap=8, loop_accept=0.55,
                 match_decim=2):
        self.res = res
        self.max_range = max_range
        self.kf_dist, self.kf_ang = kf_dist, kf_ang
        self.loop_radius, self.loop_min_gap, self.loop_accept = loop_radius, loop_min_gap, loop_accept
        self.match_decim = match_decim
        self.OFF = map_m / 2.0

        self.gm = OccupancyGridMap(size_m=(map_m, map_m), res=res)
        self.matcher = ScanMatcher(self.gm)
        self.graph = PoseGraph()
        self.keyframes = []                   # {'node', 'scan'(Nx2 로봇프레임 hit)}
        self.pose = np.array([self.OFF, self.OFF, 0.0])
        self._last_odom = None
        self.n_loops = 0

    # ---------- 스캔 → 로봇프레임 hit 점 ----------
    def _hits(self, angles, ranges):
        angles = np.asarray(angles); ranges = np.asarray(ranges)
        m = ranges < self.max_range - 1e-3
        a, r = angles[m], ranges[m]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    # ---------- 메인: 스캔 1개 처리 ----------
    def process(self, odom, angles, ranges):
        odom = np.asarray(odom, float)
        # 1) 오도메 증분 → 포즈 예측
        if self._last_odom is not None:
            self.pose = _compose(self.pose, _rel(self._last_odom, odom))
        self._last_odom = odom.copy()

        hits = self._hits(angles, ranges)
        need_kf = (not self.keyframes) or self._moved_enough()
        if not need_kf:
            return self.pose

        # 2) 키프레임 생성 전 스캔매칭으로 포즈 보정(맵이 있으면)
        if self.keyframes and len(hits) > 10:
            self.pose = np.array(self.matcher.match(tuple(self.pose),
                                                    hits[::self.match_decim]))

        # 3) 노드 + 오도메 엣지 추가
        node = self.graph.add_node(self.pose.copy())
        if self.keyframes:
            prev = self.keyframes[-1]
            z = _rel(self.graph.nodes[prev['node']], self.pose)
            self.graph.add_edge(prev['node'], node, z, np.diag([50., 50., 100.]))
        self.keyframes.append({'node': node, 'scan': hits, 'pose_at_kf': self.pose.copy()})

        # 4) 맵에 누적 + 필드 갱신
        self.gm.integrate(tuple(self.pose), np.asarray(angles), np.asarray(ranges), self.max_range)
        self.matcher.update_field()

        # 5) 루프탐지 → 있으면 최적화 + 맵 재구성
        if self._detect_loops(len(self.keyframes) - 1):
            self.graph.optimize(iterations=15)
            self._sync_from_graph()
            self._rebuild_map()

        return self.pose

    def _moved_enough(self):
        p0 = self.graph.nodes[self.keyframes[-1]['node']]
        d = math.hypot(self.pose[0] - p0[0], self.pose[1] - p0[1])
        da = abs(_wrap(self.pose[2] - p0[2]))
        return d > self.kf_dist or da > self.kf_ang

    # ---------- 루프탐지 ----------
    def _detect_loops(self, new_i):
        new = self.keyframes[new_i]
        np_pose = self.graph.nodes[new['node']]
        found = False
        for old_i in range(new_i - self.loop_min_gap + 1):
            old = self.keyframes[old_i]
            op = self.graph.nodes[old['node']]
            if math.hypot(np_pose[0] - op[0], np_pose[1] - op[1]) > self.loop_radius:
                continue
            z, score = self._match_pair(old, new)
            if score >= self.loop_accept:
                self.graph.add_edge(old['node'], new['node'], z, np.diag([80., 80., 160.]))
                self.n_loops += 1
                found = True
        return found

    def _match_pair(self, old, new):
        """old 키프레임 스캔으로 로컬맵 만들고, new 스캔을 정합 → (상대변환 z, 점수)."""
        Lm = 2 * self.max_range + 2.0
        Lc = Lm / 2.0
        g = OccupancyGridMap(size_m=(Lm, Lm), res=self.res)
        # old 스캔을 로컬중심(heading 0)에 누적
        oa = np.arctan2(old['scan'][:, 1], old['scan'][:, 0])
        orng = np.hypot(old['scan'][:, 0], old['scan'][:, 1])
        g.integrate((Lc, Lc, 0.0), oa, orng, self.max_range)
        sm = ScanMatcher(g); sm.update_field()
        # new 의 예측 상대포즈(old 프레임)
        rel = _rel(self.graph.nodes[old['node']], self.graph.nodes[new['node']])
        init = (Lc + rel[0], Lc + rel[1], rel[2])
        pts = new['scan'][::self.match_decim]
        matched = sm.match(init, pts,
                           coarse=(0.5, math.radians(25), 8),
                           fine=(0.05, math.radians(3), 4))
        # 점수 = new 점들이 로컬맵 '벽 근처'에 얹힌 비율 (스케일 무관)
        mx, my, mth = matched
        c, s = math.cos(mth), math.sin(mth)
        wx = mx + pts[:, 0] * c - pts[:, 1] * s
        wy = my + pts[:, 0] * s + pts[:, 1] * c
        cols = (wx / self.res).astype(np.int32); rows = (wy / self.res).astype(np.int32)
        m = (rows >= 0) & (rows < g.H) & (cols >= 0) & (cols < g.W)
        vals = sm.field[rows[m], cols[m]] if m.any() else np.zeros(1)
        overlap = float(np.mean(vals > 0.02))
        z = np.array([mx - Lc, my - Lc, _wrap(mth)])
        return z, overlap

    # ---------- 최적화 결과 반영 ----------
    def _sync_from_graph(self):
        # 현재 포즈를 마지막 키프레임 보정만큼 이동
        last = self.keyframes[-1]
        before = last['pose_at_kf']
        after = self.graph.nodes[last['node']]
        self.pose = _compose(after, _rel(before, self.pose))
        for kf in self.keyframes:
            kf['pose_at_kf'] = self.graph.nodes[kf['node']].copy()

    def _rebuild_map(self):
        self.gm.log[:] = 0.0
        for kf in self.keyframes:
            p = self.graph.nodes[kf['node']]
            sc = kf['scan']
            if len(sc) == 0:
                continue
            a = np.arctan2(sc[:, 1], sc[:, 0])
            r = np.hypot(sc[:, 0], sc[:, 1])
            self.gm.integrate(tuple(p), a, r, self.max_range)
        self.matcher.update_field()

    def save(self, path):
        self.gm.save(path)
