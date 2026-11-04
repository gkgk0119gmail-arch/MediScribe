"""
라이다 스캔 변환 — 원시 [(강도, 각도°, 거리mm)] 한 바퀴를 로봇 좌표로.

각도 규약(LIDAR_FLIP·ANGLE_OFFSET_DEG), 유효거리(MAX_RANGE), 차체 반사 제거(BODY_BOX_M)를
이 한 곳에서만 적용한다. 매핑과 주행이 서로 다른 규약을 쓰면 지도와 위치가 조용히 어긋나기 때문이다.
"""
from __future__ import annotations

import math

import numpy as np

from ..config import MAX_RANGE, LIDAR_FLIP, ANGLE_OFFSET_DEG, BODY_BOX_M

_BX0, _BX1, _BY0, _BY1 = BODY_BOX_M


def _in_body(x, y):
    """로봇프레임 점이 차 몸통 박스 안? (자기 차 반사 = 실제 장애물일 수 없음)"""
    return _BX0 < x < _BX1 and _BY0 < y < _BY1


def scan_to_points(scan, max_range=MAX_RANGE):
    """한 바퀴 → 로봇 좌표 점 Nx2 [m] (x 앞, y 왼쪽). MAX_RANGE 밖과 차체 반사는 버린다.
    점 순서는 측정 순서를 유지한다."""
    off = math.radians(ANGLE_OFFSET_DEG)
    pts = []
    for _q, a, d in scan:
        if d <= 0:
            continue
        dm = d / 1000.0
        if dm >= max_range:
            continue
        phi = math.radians(a) * LIDAR_FLIP + off
        x, y = dm * math.cos(phi), dm * math.sin(phi)
        if _in_body(x, y):
            continue                                  # 자기 차 반사 제거
        pts.append((x, y))
    return np.array(pts) if pts else np.zeros((0, 2))


def scan_to_rays(scan):
    """한 바퀴 → (angles[rad], ranges[m]). SLAM·주행이 쓰는 형태.

    scan_to_points 와 같은 각도 규약이다. 여기는 MAX_RANGE 로 자르지 않는다 —
    먼 반사도 점유격자에 '빈 공간'을 새기는 데 쓰이기 때문이다(거리 제한은 mapping.integrate 가 한다)."""
    off = math.radians(ANGLE_OFFSET_DEG)
    angs, rngs = [], []
    for _q, a, d in scan:
        if d <= 0:
            continue
        phi = math.radians(a) * LIDAR_FLIP + off; dm = d / 1000.0
        if _in_body(dm * math.cos(phi), dm * math.sin(phi)):
            continue                                  # 자기 차 반사 제거 (free-space 새김에도 무의미)
        angs.append(phi)
        rngs.append(dm)
    return np.array(angs), np.array(rngs)
