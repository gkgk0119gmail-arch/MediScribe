"""
자이로 요각 적분기 — 요레이트를 적분해 헤딩 변화를 '측정'한다.

왜: 엔코더 없는 차는 회전량을 라이다 정합으로만 추정한다. 급회전에서는 스캔 사이 회전이 커서
정합이 따라가기 어렵다. 자이로는 회전을 직접 재므로 정합의 초기값과 검증에 쓴다.
대신 정지 중 바이어스가 흐르므로 계속 추적한다.

바이어스: 시작 1초 정지 평균으로 초기화하고, 소비자가 set_stationary() 로 알려 주는 '정지 확실'
신호와 자체 정지 감지(레이트가 좁은 띠에 2초 머무름)로 계속 갱신한다.
적분 dt 는 하드웨어 타임스탬프 차분이다 — 도착 시각 지터를 타지 않는다.
센서 리더는 hal/imu_iio.py (D435i, Linux iio).
"""
from __future__ import annotations

import threading
import time
from bisect import bisect_left
from collections import deque

from ..config import IMU_GYRO_HZ, IMU_STALE_S


class GyroYaw:
    """순수 요각 적분기 (테스트 대상 — 하드웨어 무관).

    add() 로 (하드웨어시각[s], 보정전 레이트[rad/s], 벽시계[s]) 를 넣으면
    바이어스를 빼며 적분하고, (벽시계, 누적요각) 링버퍼를 유지한다.
    적분 dt 는 하드웨어 타임스탬프 차분 — 도착시각 지터를 안 탄다.
    """

    def __init__(self, hz=IMU_GYRO_HZ, bias_init_s=1.0, horizon_s=10.0,
                 still_band=0.02, still_hold_s=2.0, bias_abs_max=0.1):
        self.bias = 0.0
        self._init_n = max(1, int(hz * bias_init_s))   # 초기 바이어스 표본수(1초 창)
        self._init_sum = 0.0
        self._init_cnt = 0
        self._init_lo = float("inf")                   # 초기 창의 표본 범위 — 움직임 감지용
        self._init_hi = float("-inf")
        self.yaw = 0.0                                 # 누적 요각 [rad] (unwrap — 안 접는다)
        self._t_hw = None                              # 직전 하드웨어 시각 [s]
        self._buf = deque(maxlen=int(hz * horizon_s))  # (벽시계, yaw) — delta() 조회용
        self._last_wall = 0.0
        self._last_raw = None
        # ★바이어스 게이트는 전부 '절대값' 기준이다 — 현재 추정치 중심의 띠로 걸면,
        #   한 번 오염된 추정치가 참값(≈0 부근)을 영원히 띠 밖으로 밀어내 복구가
        #   수학적으로 불가능해진다(리뷰에서 실증). MEMS 요 바이어스는 물리적으로
        #   |bias| ≤ ~0.03 rad/s 급이므로 |raw| < bias_abs_max(0.1) 가 타당성 게이트.
        self._still_band = still_band
        self._still_need = max(1, int(hz * still_hold_s))
        self._still_run = 0
        self._raw_ema = 0.0                            # 최근 레이트 평균 — '안정' 판정 기준
        self._bias_abs_max = bias_abs_max
        self._stationary = False                       # 소비자가 주는 '정지 확실' 신호

    def set_stationary(self, flag):
        self._stationary = bool(flag)

    def add(self, t_hw, rate_raw, wall_t):
        """샘플 1개 반영. rate_raw 는 축매핑/부호까지 끝난 로봇 요레이트(보정 전)."""
        if self._init_cnt < self._init_n:
            # 초기 바이어스 수집 — '정지 가정'을 검증한다: 창 안 표본 범위가 넓거나
            # (움직이는 중) 평균이 물리적 바이어스 범위를 벗어나면(등속 회전 중) 그 창을
            # 버리고 다시 모은다. 오염된 초기값은 이후 모든 헤딩을 틀리게 하는데, 열리는
            # 시점을 코드가 못 고르므로(감독 스레드가 아무 때나 연다) 검증이 필수.
            self._init_sum += rate_raw
            self._init_cnt += 1
            self._init_lo = min(self._init_lo, rate_raw)
            self._init_hi = max(self._init_hi, rate_raw)
            mean = self._init_sum / self._init_cnt
            if self._init_cnt >= self._init_n:
                if (self._init_hi - self._init_lo > 2.0 * self._still_band or
                        abs(mean) > self._bias_abs_max):
                    self._init_sum = 0.0               # 창 폐기 — 정지가 올 때까지 재수집
                    self._init_cnt = 0
                    self._init_lo, self._init_hi = float("inf"), float("-inf")
                else:
                    self.bias = mean
                    self._raw_ema = mean
            self._t_hw = t_hw
            self._last_wall, self._last_raw = wall_t, rate_raw
            # ★초기화 중엔 링버퍼를 채우지 않는다 — 채우면 delta() 가 그 구간을 '회전 0'
            #   으로 확신 반환해, 초기화 중 실회전을 소비자가 정합 dθ 대신 0 으로 치환한다
            #   (리뷰에서 실증). 버퍼가 비어 있으면 delta() 는 정직하게 None = 모른다.
            return
        # 바이어스 갱신 — '정지 확실'(강한 신호) 또는 자체 정지감지(약한 신호)일 때만.
        # 자체 감지 = 안정(최근 평균 ±still_band 유지) + 타당(|raw| < bias_abs_max).
        # ★타당성 게이트 덕에 손밀기·주행 회전(|raw| 큼)으로는 절대 안 갱신되고,
        #   참 바이어스(작음)로는 언제든 되돌아올 수 있다 — 갱신을 놓치는 쪽이 덜 나쁘다.
        plausible = abs(rate_raw) < self._bias_abs_max
        stable = abs(rate_raw - self._raw_ema) < self._still_band
        self._raw_ema += 0.05 * (rate_raw - self._raw_ema)
        self._still_run = self._still_run + 1 if (stable and plausible) else 0
        if plausible and (self._stationary or self._still_run >= self._still_need):
            alpha = 0.05 if self._stationary else 0.01
            self.bias += alpha * (rate_raw - self.bias)
        if self._t_hw is not None:
            dt = t_hw - self._t_hw
            if 0.0 < dt < 0.05:                        # 결번/타임스탬프 점프는 그 구간 버림
                self.yaw += (rate_raw - self.bias) * dt
        self._t_hw = t_hw
        self._last_wall, self._last_raw = wall_t, rate_raw
        self._buf.append((wall_t, self.yaw))

    def rate(self, now=None, max_age=IMU_STALE_S):
        """최신 보정 레이트 [rad/s]. 데이터 없음/오래됨(스트림 중단)/초기화 미완 → None."""
        if self._last_raw is None or self._init_cnt < self._init_n:
            return None
        if ((now if now is not None else time.time()) - self._last_wall) > max_age:
            return None
        return self._last_raw - self.bias

    def delta(self, t0, t1, edge_tol=0.15):
        """벽시계 [t0, t1] 사이 요각 변화 [rad]. 버퍼가 그 구간을 못 덮으면 None.

        edge_tol: 버퍼 양끝 허용 오차 [s] — 오도메 샘플시각과 자이로 도착시각의
        어긋남(수 ms)+한 틱 분량을 흡수. 그 밖이면 '모른다'가 정답(억지 외삽 금지)."""
        if t0 is None or t1 is None or not self._buf:
            return None
        buf = list(self._buf)
        if t0 < buf[0][0] - edge_tol or t1 > buf[-1][0] + edge_tol:
            return None
        ts = [b[0] for b in buf]

        def _yaw_at(t):
            i = bisect_left(ts, t)
            if i <= 0:
                return buf[0][1]
            if i >= len(buf):
                return buf[-1][1]
            (ta, ya), (tb, yb) = buf[i - 1], buf[i]
            f = 0.0 if tb <= ta else (t - ta) / (tb - ta)
            return ya + f * (yb - ya)

        return _yaw_at(t1) - _yaw_at(t0)
