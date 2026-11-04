"""
로컬라이제이션 복구 탐색 — 추적을 놓쳤을 때(저신뢰 연속) 단계적으로 되찾는다.

왜: 스캔 정합 추적은 시드 주변 좁은 창(주행 중 ±0.18m/±11°)만 본다.
차를 손으로 들어 옮기거나 바퀴가 헛돌아 실제 위치가 창을 벗어나면, 정합이 낮은 점수의
국소최적만 맴돈다. 여기서 그 복구를 자동으로 한다(사람이 위치를 다시 찍지 않아도 된다).

단계 (싼 것부터 — 상실 원인의 빈도 순):
  1) 헤딩 스윕: 제자리 회전을 놓친 경우가 압도적(손으로 돌리기·슬립 회전).
     (x,y) 고정 후 0~360° 훑기 — _score 벡터화라 180회 평가 ≈ 수 ms.
  2) 지역 격자: 반경 radius_m 안 (x,y)×θ 코스 탐색 후 정밀 매칭.
     제어 틱 예산(50ms)을 넘길 수 있으나 '위치를 모른 채 주행'이 항상 더 나쁘다 —
     호출측이 LOC_RECOVER_PERIOD_S 로 빈도를 제한한다.

전역 탐색(맵 전체)은 여기 두지 않는다 — 초 단위라 주행 루프에서 못 돌린다.
그 경우는 뷰어에서 위치를 수동으로 찍는다(nav_drive 뷰어: 오른쪽 클릭+끌기).
"""
from __future__ import annotations

import math

import numpy as np

# 복구 후 정밀화 창: 코스 탐색 격자(0.25m/10°)가 남긴 오차를 덮는 크기.
_REFINE = (0.3, math.radians(8.0), 6)
# 슬립/상실 시 추적 매칭에 쓰는 넓은 창 (오도메 예측을 못 믿는 상황).
WIDE_COARSE = (0.5, math.radians(25.0), 10)


def match_conf(matcher, pose, pts):
    """정합 신뢰도 = 점당 거리장 점수의 평균 (0~1)."""
    if len(pts) == 0:
        return 0.0
    return float(matcher._score(pose[0], pose[1], pose[2], pts) / len(pts))


def heading_sweep(matcher, pose, pts, step_deg=2.0):
    """1단계: (x,y) 고정, 헤딩만 0~360° 스윕 → 최고점에서 정밀 매칭.

    step 2° 근거: 정밀 매칭 창(±8°)이 이웃 스텝 사이를 확실히 덮는 간격."""
    x, y, _ = pose
    best_th, best_s = pose[2], -1.0
    for th in np.arange(0.0, 2.0 * math.pi, math.radians(step_deg)):
        s = matcher._score(x, y, th, pts)
        if s > best_s:
            best_s, best_th = s, float(th)
    cand = matcher.match((x, y, best_th), pts, coarse=_REFINE)
    return tuple(cand), match_conf(matcher, cand, pts)


def local_search(matcher, pose, pts, radius_m=2.0, step_m=0.25, head_step_deg=10.0):
    """2단계: 반경 radius_m 지역 격자 (x,y)×θ 코스 탐색 → 최고점에서 정밀 매칭.

    격자 근거: step 0.25m/10° 는 정밀화 창(±0.3m/±8°)이 이웃 스텝을 덮는 최대 간격.
    비용: (2·2/0.25+1)²=289 병진 × 36 헤딩 — _score_grid 벡터화로 수십 ms."""
    x, y, _ = pose
    xs = np.arange(x - radius_m, x + radius_m + 1e-9, step_m)
    ys = np.arange(y - radius_m, y + radius_m + 1e-9, step_m)
    best, best_s = tuple(pose), -1.0
    for th in np.arange(0.0, 2.0 * math.pi, math.radians(head_step_deg)):
        X, Y, scores = matcher._score_grid(xs, ys, float(th), pts)
        k = int(scores.argmax())
        if scores[k] > best_s:
            best_s, best = float(scores[k]), (float(X[k]), float(Y[k]), float(th))
    cand = matcher.match(best, pts, coarse=_REFINE)
    return tuple(cand), match_conf(matcher, cand, pts)


def try_recover(matcher, pose, pts, level):
    """상실 지속도에 따른 단계 선택. level 1=헤딩 스윕, 2부터=지역 격자.

    반환 (복구후보 pose, 신뢰도) — 채택 여부는 호출측이 LOC_RECOVER_MIN 으로 판단
    (여기서 임계값을 갖지 않는 이유: 추적/저장 게이트와 한 곳(config)에서 관리)."""
    if len(pts) == 0:
        return tuple(pose), 0.0
    if level <= 1:
        return heading_sweep(matcher, pose, pts)
    return local_search(matcher, pose, pts)
