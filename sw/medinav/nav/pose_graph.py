"""
2D 포즈그래프 최적화 (그래프 기반 SLAM의 백엔드) — 순수 파이썬/numpy.

노드 = 로봇 포즈 (x, y, θ),  엣지 = 두 포즈 사이의 상대변환 제약 (측정 + 정보행렬).
모든 제약을 가장 잘 만족하는 포즈들을 Gauss-Newton 비선형최소제곱으로 찾는다.
→ 루프클로저 엣지 하나가 들어오면, 누적 드리프트를 전체 궤적에 분산해서 "닫음".

참고: Grisetti et al., "A Tutorial on Graph-Based SLAM"의 SE(2) 정식화.
"""
from __future__ import annotations
import math
import numpy as np

from ..core.se2 import v2t, t2v, wrap as _wrap   # SE(2) 유틸 정본


class PoseGraph:
    def __init__(self):
        self.nodes = []                 # list of (x,y,θ)
        self.edges = []                 # list of (i, j, z(3,), Omega(3x3))

    def add_node(self, pose):
        self.nodes.append(np.asarray(pose, dtype=float))
        return len(self.nodes) - 1

    def add_edge(self, i, j, z, omega=None):
        if omega is None:
            omega = np.eye(3)
        self.edges.append((i, j, np.asarray(z, float), np.asarray(omega, float)))

    def _error_and_jac(self, xi, xj, z):
        """엣지 오차 e 와 야코비안 A(=∂e/∂xi), B(=∂e/∂xj)."""
        Zt, Xi, Xj = v2t(z), v2t(xi), v2t(xj)
        e = t2v(np.linalg.inv(Zt) @ np.linalg.inv(Xi) @ Xj)
        e[2] = _wrap(e[2])

        si, ci = math.sin(xi[2]), math.cos(xi[2])
        RiT = np.array([[ci, si], [-si, ci]])
        dRiT = np.array([[-si, ci], [-ci, -si]])       # d(Ri^T)/dθi
        cz, sz = math.cos(z[2]), math.sin(z[2])
        RijT = np.array([[cz, sz], [-sz, cz]])
        dt = np.array([xj[0] - xi[0], xj[1] - xi[1]])

        A = np.zeros((3, 3)); B = np.zeros((3, 3))
        A[0:2, 0:2] = -RijT @ RiT
        A[0:2, 2] = RijT @ dRiT @ dt
        A[2, 2] = -1.0
        B[0:2, 0:2] = RijT @ RiT
        B[2, 2] = 1.0
        return e, A, B

    def total_error(self):
        tot = 0.0
        for i, j, z, om in self.edges:
            e, _, _ = self._error_and_jac(self.nodes[i], self.nodes[j], z)
            tot += float(e @ om @ e)
        return tot

    def optimize(self, iterations=20, tol=1e-6, verbose=False):
        n = len(self.nodes)
        N = 3 * n
        for it in range(iterations):
            H = np.zeros((N, N))
            b = np.zeros(N)
            for i, j, z, om in self.edges:
                e, A, B = self._error_and_jac(self.nodes[i], self.nodes[j], z)
                ii, jj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
                H[ii, ii] += A.T @ om @ A
                H[ii, jj] += A.T @ om @ B
                H[jj, ii] += B.T @ om @ A
                H[jj, jj] += B.T @ om @ B
                b[ii] += A.T @ om @ e
                b[jj] += B.T @ om @ e
            H[0:3, 0:3] += np.eye(3) * 1e6      # 첫 노드 고정(게이지 자유도 제거)
            dx = np.linalg.solve(H, -b)
            for k in range(n):
                self.nodes[k] = self.nodes[k] + dx[3 * k:3 * k + 3]
                self.nodes[k][2] = _wrap(self.nodes[k][2])
            step = float(np.linalg.norm(dx))
            if verbose:
                print(f"  iter {it}: |dx|={step:.3e}  err={self.total_error():.4e}")
            if step < tol:
                break
        return self.total_error()
