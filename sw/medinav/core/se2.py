"""
SE(2) 포즈 연산 — (x, y, theta) 2D 강체변환.

포즈는 (x, y, theta) 로 인덱싱 가능한 것(tuple/list/ndarray, theta[rad])이고 반환은 np.ndarray.
그래프 SLAM, 스캔 정합, 주행 제어가 같은 구현을 공유한다.
"""
from __future__ import annotations
import math
import numpy as np


def wrap(a):
    """각도를 [-pi, pi) 로 정규화. (구 _wrap — pose_graph/graph_slam 공통)"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def v2t(v):
    """(x, y, theta) → 3x3 동차변환. (구 pose_graph.v2t)"""
    c, s = math.cos(v[2]), math.sin(v[2])
    return np.array([[c, -s, v[0]], [s, c, v[1]], [0.0, 0.0, 1.0]])


def t2v(T):
    """3x3 동차변환 → (x, y, theta). (구 pose_graph.t2v)"""
    return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])


def rel(a, b):
    """a→b 상대변환 (a 프레임 기준). """
    return t2v(np.linalg.inv(v2t(a)) @ v2t(b))


def compose(a, rel):
    """a 에 상대변환 rel 합성. """
    return t2v(v2t(a) @ v2t(rel))
