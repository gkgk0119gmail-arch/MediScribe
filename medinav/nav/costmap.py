"""
Costmap — 로봇을 '점'으로 보고 장애물을 로봇 반지름만큼 부풀린(inflate) 격자.
A* 가 이 위에서 경로를 짠다.

2층 구조(ROS costmap 방식):
  · 치명존(lethal)   : 벽에서 robot_radius 이내 → 하드 블록(로봇 중심이 못 감).
                       robot_radius 는 주행 코드가 차폭/2 + 여유로 정한다(scripts/nav_drive.py --car-halfw).
  · 팽창존(inflation): 치명존 밖 ~ inflation_m 까지 → 거리에 따라 감소하는 '소프트 비용'.
                       A* 가 지나갈 순 있지만 비싸므로 벽에서 넉넉히 떨어져 선제 우회.

self.grid : bool  True=하드블록(치명존). is_free/free_world/augmented 는 이걸 본다.
self.cost : float 0~1  A* 가 추가로 무는 비용(치명존=1, 팽창존 지수감소, 그 밖=0).
"""
from __future__ import annotations
import numpy as np


def _dilate1(g: np.ndarray) -> np.ndarray:
    """8-이웃 1셀 팽창."""
    out = g.copy()
    out[1:, :] |= g[:-1, :]
    out[:-1, :] |= g[1:, :]
    out[:, 1:] |= g[:, :-1]
    out[:, :-1] |= g[:, 1:]
    out[1:, 1:] |= g[:-1, :-1]
    out[1:, :-1] |= g[:-1, 1:]
    out[:-1, 1:] |= g[1:, :-1]
    out[:-1, :-1] |= g[1:, 1:]
    return out


def inflate(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    g = grid.astype(bool).copy()
    for _ in range(max(0, radius_cells)):
        g = _dilate1(g)
    return g


def dist_layers(base: np.ndarray, max_cells: int) -> np.ndarray:
    """장애물로부터의 (체비셰프) 거리 층. 0=벽, k=k셀 떨어짐, 그 밖=max_cells+1."""
    base = base.astype(bool)
    dist = np.full(base.shape, max_cells + 1, dtype=np.int32)
    dist[base] = 0
    cur = base.copy()
    for k in range(1, max_cells + 1):
        nxt = _dilate1(cur)
        newly = nxt & ~cur
        dist[newly] = k
        cur = nxt
    return dist


def _cost_from_dist(dist: np.ndarray, lethal_cells: int, inflation_cells: int,
                    cost_scale: float) -> np.ndarray:
    """거리층 → 소프트비용(0~1). 치명존=1, 팽창존 지수감소, 그 밖=0."""
    cost = np.zeros(dist.shape, float)
    cost[dist <= lethal_cells] = 1.0
    band = (dist > lethal_cells) & (dist <= inflation_cells)
    span = max(1, inflation_cells - lethal_cells)
    t = (dist[band] - lethal_cells) / span          # 0(치명존 경계) ~ 1(팽창존 끝)
    cost[band] = np.exp(-cost_scale * t)            # 경계에서 1 → 지수 감소
    return cost


class Costmap:
    def __init__(self, grid: np.ndarray, res: float,
                 robot_radius_m: float = 0.42, inflation_m: float = 0.55,
                 cost_scale: float = 3.0):
        self.res = float(res)
        self.base = grid.astype(bool)
        self.H, self.W = self.base.shape
        self.cost_scale = cost_scale

        self.lethal_cells = int(round(robot_radius_m / res))
        self.inflation_cells = self.lethal_cells + int(round(inflation_m / res))

        dist = dist_layers(self.base, self.inflation_cells)
        self.grid = dist <= self.lethal_cells          # True=하드블록(치명존)
        self.cost = _cost_from_dist(dist, self.lethal_cells,
                                    self.inflation_cells, cost_scale)

    # 월드(m) ↔ 셀(row=y, col=x)
    def world_to_cell(self, x, y):
        return (int(round(y / self.res)), int(round(x / self.res)))  # (row, col)

    def cell_to_world(self, cell):
        r, c = cell
        return (c * self.res, r * self.res)

    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.H and 0 <= c < self.W

    def is_free(self, cell):
        return self.in_bounds(cell) and not self.grid[cell[0], cell[1]]

    def free_world(self, x, y):
        return self.is_free(self.world_to_cell(x, y))

    def cell_cost(self, cell):
        """A* 가 추가로 무는 소프트비용(0~1). 범위 밖=0."""
        r, c = cell
        if 0 <= r < self.H and 0 <= c < self.W:
            return self.cost[r, c]
        return 0.0

    def augmented(self, extra_grid, extra_cost=None):
        """정적 맵 + 추가 장애물(예: 라이다로 본 사람)을 합친 임시 costmap.
        재계획(A*)이 동적 장애물을 피해가게 할 때 쓴다.
        extra_cost 를 주면 동적 장애물 주변 소프트비용도 합친다(max)."""
        c = Costmap.__new__(Costmap)
        c.res = self.res
        c.base = self.base
        c.H, c.W = self.H, self.W
        c.cost_scale = self.cost_scale
        c.lethal_cells = self.lethal_cells
        c.inflation_cells = self.inflation_cells
        c.grid = self.grid | np.asarray(extra_grid, bool)
        if extra_cost is not None:
            c.cost = np.maximum(self.cost, np.asarray(extra_cost, float))
        else:
            c.cost = self.cost
        c.cost = np.where(c.grid, 1.0, c.cost)         # 하드블록은 비용 1
        return c
