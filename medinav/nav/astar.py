"""
A* 글로벌 플래너 — costmap(팽창격자) 위에서 시작→목표 경로.

8-방향 이동(대각선 코너 컷 방지), 휴리스틱=유클리드.
치명존(하드블록)은 못 지나가고, 팽창존 소프트비용은 이동비용에 더해
벽에서 멀찍이(=비용 낮은 쪽) 넉넉히 우회하는 경로를 짠다.
결과를 직선가시(line-of-sight)로 스무딩해 지그재그를 줄인다.
반환: 월드좌표 [(x,y), ...] 또는 None(경로 없음).
"""
from __future__ import annotations
import heapq
import math
import time

# 소프트비용 가중치: 셀당 이동비용(1) 대비 벽근접 페널티 배율.
# 클수록 벽에서 더 멀리 돌아감(선제 우회 강해짐). 낮추면 더 직진/타이트.
COST_WEIGHT = 1.6                                  # ↓2.5→1.6: 좁아도 통과가능하면 빙 안돌고 지나감

_SQRT2 = math.sqrt(2.0)
# (dr, dc, cost)
_NEI = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
        (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]


def _nearest_free(costmap, cell, max_r=20):
    """cell 이 막혀있으면 근처 빈 셀로 옮긴다."""
    if costmap.is_free(cell):
        return cell
    r0, c0 = cell
    for rad in range(1, max_r):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                cand = (r0 + dr, c0 + dc)
                if costmap.is_free(cand):
                    return cand
    return None


def _line_free(costmap, a, b, max_cost=0.15):
    """월드 두 점 사이 직선이 전부 free 이고, 소프트비용도 낮은지(샘플링).
    max_cost 초과 셀을 지나면 단축 불가 → cost-aware 우회가 스무딩에 안 지워짐."""
    (ax, ay), (bx, by) = a, b
    dist = math.hypot(bx - ax, by - ay)
    n = max(1, int(dist / (costmap.res * 0.5)))
    cellcost = getattr(costmap, "cell_cost", None)
    for i in range(n + 1):
        t = i / n
        x, y = ax + (bx - ax) * t, ay + (by - ay) * t
        if not costmap.free_world(x, y):
            return False
        if cellcost is not None and cellcost(costmap.world_to_cell(x, y)) > max_cost:
            return False
    return True


def _smooth(costmap, pts):
    """LOS 단축: 멀리 보이는 점으로 바로 이어 웨이포인트 줄임(비싼 구역은 안 가로지름)."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _line_free(costmap, pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def astar(costmap, start_xy, goal_xy, cost_weight=COST_WEIGHT,
          time_limit=1.0, max_expand=120000):
    start = _nearest_free(costmap, costmap.world_to_cell(*start_xy))
    goal = _nearest_free(costmap, costmap.world_to_cell(*goal_xy))
    if start is None or goal is None:
        return None
    t0 = time.perf_counter()

    # 소프트비용 지원 costmap 이면 벽근접 페널티를 이동비용에 더한다(구버전이면 0).
    cellcost = getattr(costmap, "cell_cost", None)

    def h(c):
        return math.hypot(c[0] - goal[0], c[1] - goal[1])

    open_h = [(h(start), 0.0, start)]
    came = {}
    gcost = {start: 0.0}
    seen = set()
    while open_h:
        _, g, cur = heapq.heappop(open_h)
        if cur in seen:
            continue
        seen.add(cur)
        # ★도달불가·과탐색 방지 — 상한 넘으면 빨리 포기(맵 전체 뒤지다 수초 프리즈하는 것 근절)
        if len(seen) > max_expand or (len(seen) & 0x1FFF) == 0 and \
                time.perf_counter() - t0 > time_limit:
            return None
        if cur == goal:
            break
        r, c = cur
        for dr, dc, cost in _NEI:
            nb = (r + dr, c + dc)
            if not costmap.is_free(nb):
                continue
            if dr != 0 and dc != 0:  # 대각선 코너 컷 방지
                if not (costmap.is_free((r + dr, c)) and costmap.is_free((r, c + dc))):
                    continue
            step = cost
            if cellcost is not None:
                step += cost_weight * cellcost(nb)   # 벽근접일수록 비쌈 → 넉넉히 우회
            ng = g + step
            if ng < gcost.get(nb, float("inf")):
                gcost[nb] = ng
                came[nb] = cur
                heapq.heappush(open_h, (ng + h(nb), ng, nb))

    if goal not in came and goal != start:
        return None

    # 경로 복원(셀→월드)
    cells = [goal]
    while cells[-1] != start:
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = [costmap.cell_to_world(c) for c in cells]
    return _smooth(costmap, pts)
