#!/usr/bin/env python3
"""
저장한 지도 위 자율주행 — 엔코더 없는 애커만 차량(조향각 센서 없는 DC 조향) + STL-27L + D435i 자이로.

스캔(10 Hz)마다:
  위치추정: 라이다 → 저장 지도에 scan-to-map 정합(매핑과 같은 방식), 신뢰도 게이트 + 상실 시 자동 재탐색
  경로:     목표까지 A*(차폭 기준으로 부풀린 코스트맵) → 전방 주시점 → 헤딩오차 → 목표 요레이트
  조향:     정합으로 잰 실제 요레이트와 비교해 조향 펄스(좌/우) — 조향각 센서 없이 직진을 복원
  속도:     앞범퍼 거리로 감속/정지
  후진 기동: 애커만은 제자리 회전이 안 된다 → 목표가 뒤쪽이면 먼저 후진하며 반대로 꺾고,
            앞이 막히면 뒤가 빌 때 후진 탈출. 뒤범퍼 여유가 줄면 즉시 멈춘다.
차체 치수(--car-front/--car-rear/--car-halfw)로 모든 거리 기준을 범퍼 기준으로 계산한다.

목표: 뷰어 http://<젯슨>:8766 에서 지도 클릭(POST /goal). 오른쪽 클릭+끌기 = 차 위치/방향 지정(POST /init).
시작 위치: 마지막 위치 기억(같은 지도일 때) 또는 --init x y deg, 이어서 헤딩 360° 스윕 + 반경 탐색으로 보정.
정지: 조이스틱 B/Back, 뷰어 s 키(POST /stop), Ctrl+C. 재개: 뷰어 g 키(POST /go).

실행 (sw/ 에서):  python3 scripts/nav_drive.py --map out/joymap_<시각>.npz --pwm 120
"""
import signal as _sig
_sig.signal(_sig.SIGINT, _sig.default_int_handler)      # &로 띄우면 SIGINT 가 무시로 상속됨 → 복구
def _term(*_): raise KeyboardInterrupt
_sig.signal(_sig.SIGTERM, _term)                         # stop_*.sh 의 TERM 도 정상종료(finally 저장) 경로로
import os, sys, io, time, math, glob, json, select, struct, argparse, threading
from urllib.parse import urlparse, parse_qs
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)
from medinav.hal.lidar_stl27l import STL27L                      # noqa: E402
from medinav.hal.lidar import scan_to_rays                        # noqa: E402
from medinav.nav.mapping import OccupancyGridMap                  # noqa: E402
from medinav.nav.scan_match import ScanMatcher                    # noqa: E402
from medinav.nav.costmap import Costmap                           # noqa: E402
from medinav.nav.astar import astar                               # noqa: E402
from medinav.nav.relocalize import heading_sweep, local_search, WIDE_COARSE    # noqa: E402
from medinav.core.se2 import rel as _rel, compose as _compose, wrap as _wrap   # noqa: E402
from medinav.config import OCC_THRESH, MAX_RANGE, MATCH_DECIM     # noqa: E402
from medinav.hal.imu_iio import open_iio_gyro                    # noqa: E402  D435i 자이로(iio) 헤딩 융합
from medinav.io.lastpose import save_last_pose, load_last_pose, loc_ok_to_save  # noqa: E402  마지막 위치 기억

EV = struct.Struct("IhBB")
AX_LX, AX_LT, AX_RT = 0, 2, 5
BT_A, BT_B, BT_Y, BT_RB, BT_BACK, BT_START = 0, 1, 3, 5, 6, 7


def find_js():
    hits = [h for h in sorted(glob.glob("/dev/input/by-id/*-joystick")) if "event" not in h] or sorted(glob.glob("/dev/input/js*"))
    return hits[0] if hits else None


class JoyThread:
    """조이스틱 50Hz 전용 스레드 — 스캔루프(10Hz, 스캔매칭 뒤)에 묶여 있던 수동조종의 ~0.1~0.2s 지연 제거.
    RB(데드맨) 잡은 동안 직접 모터 명령, 놓는 즉시 정지. B=목표해제 정지, Back=종료, Y/A=수동 최대PWM ±20."""
    def __init__(self, fd, car, nav, vman, hz=50.0):
        self.fd, self.car, self.nav, self.vman = fd, car, nav, vman
        self.manual = False; self.quit = False
        self.axes = [0.0] * 8; self.btn = [0] * 12
        self.period = 1.0 / hz
        if fd is not None:
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        was = False
        while not self.quit:
            try:
                while True:
                    d = os.read(self.fd, EV.size)
                    if len(d) < EV.size: break
                    _, val, typ, num = EV.unpack(d); typ &= 0x7F
                    if typ == 2 and num < 8: self.axes[num] = val / 32767.0
                    elif typ == 1 and num < 12:
                        self.btn[num] = val
                        if val and num == BT_BACK: self.quit = True
                        if val and num == BT_B:
                            with self.nav.lock: self.nav.enabled = False; self.nav.goal = None; self.nav.path = []
                            print("\n조이스틱 B → 자율주행 정지(목표 해제)", flush=True)
                        if val and num == BT_Y: self.vman = min(255, self.vman + 20)
                        if val and num == BT_A: self.vman = max(60, self.vman - 20)
            except BlockingIOError:
                pass
            except OSError:
                break                                               # 종료 시 fd 닫힘
            manual = bool(self.btn[BT_RB])
            if manual and self.car:
                rt = (self.axes[AX_RT] + 1) / 2; lt = (self.axes[AX_LT] + 1) / 2
                mp = int(round((rt - lt) * self.vman))
                if abs(mp) < 15: mp = 0
                elif abs(mp) < 90: mp = 90 if mp > 0 else -90
                lx = self.axes[AX_LX]; ms_ = (255 if lx > 0 else -255) if abs(lx) > 0.15 else 0
                self.car.drive(mp)
                if ms_: self.car.steer_pulse(ms_, 150)
                with self.nav.lock: self.nav.pwm, self.nav.steer = mp, ms_
            elif was and self.car:
                self.car.drive(0)                                   # 데드맨 놓는 즉시 정지(다음 스캔 안 기다림)
                with self.nav.lock: self.nav.pwm, self.nav.steer = 0, 0
            was = manual; self.manual = manual
            time.sleep(self.period)


def densify(path, step=0.1):
    out = []
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        d = math.hypot(x1 - x0, y1 - y0); n = max(1, int(d / step))
        for i in range(n):
            t = i / n; out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    out.append(tuple(path[-1])); return out


class Nav:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        gm = OccupancyGridMap.load(a.map)
        self.gm, self.res = gm, gm.res
        self.map_m = gm.W * gm.res
        self.matcher = ScanMatcher(gm); self.matcher.update_field()
        self.wall = gm.prob() > OCC_THRESH
        self.cm = Costmap(self.wall, gm.res, robot_radius_m=a.radius, inflation_m=a.inflation)
        self.pose = np.array([a.init[0], a.init[1], math.radians(a.init[2])], float)
        self.prev_pose = self.pose.copy()
        self.conf = 0.0; self.lost = 0; self.yaw_rate = 0.0; self.speed_est = 0.0
        self.goal = None; self.path = []; self.path_t = 0.0; self.lookahead = None
        self.mode = "idle"        # idle / drive / blocked / arrived / noplan / lost
        self.enabled = True       # /stop → False, /go → True
        self.pwm = 0; self.steer = 0; self.clear = float("inf"); self.hdg_err = 0.0
        self.n_scan = 0; self.ms = 0.0; self.hz = 0.0; self.err = ""
        self.trail = []
        self._png = b""
        self.last_pts = None; self.imu = None; self.t_scan = None   # 자이로 리더, 직전 스캔 벽시계
        self.rear_clear = float("inf")       # 후방 클리어런스 (탈출 후진용)
        self.escape_until = 0.0; self.escape_end = 0.0; self.escape_n = 0   # 탈출 상태
        # 후진 기동 상태 (11/04): rev_kind = "escape"(앞 막힘) | "turn"(목표가 뒤쪽) | None
        self.rev_kind = None; self.rev_min_until = 0.0; self.rev_max_until = 0.0; self.rev_cool = 0.0
        self.goal_th = None                  # 목표 도착 방향[rad] (None=방향 무관)
        self.align_fwd = True; self.align_switch = 0.0   # K턴 정렬 상태(전진/후진 교대)

    # ---------- 측위 ----------
    def init_localize(self, pts):
        """--init 위치에서 헤딩 스윕 → 반경 탐색 → 정밀 정합."""
        g = tuple(self.pose)
        cands = []
        # A) 추정 포즈 주변 넓은 창(±0.5m/±25°) 정합 — 초기 추정이 대략 맞을 때 가장 정확
        pa = self.matcher.match(g, pts, coarse=WIDE_COARSE); cands.append(("guess", pa))
        # B) 헤딩 스윕 → 지역 격자 탐색 — 초기 추정이 틀렸을 때
        pb, _ = heading_sweep(self.matcher, g, pts)
        pb, _ = local_search(self.matcher, pb, pts, radius_m=self.a.init_radius, step_m=0.2, head_step_deg=10.0)
        pb = self.matcher.match(tuple(pb), pts); cands.append(("search", pb))
        scored = [(self.matcher._score(p[0], p[1], p[2], pts) / max(1, len(pts)), name, p) for name, p in cands]
        scored.sort(reverse=True)
        self.init_report = " | ".join(f"{n}:({p[0]-self.map_m/2:+.2f},{p[1]-self.map_m/2:+.2f},{math.degrees(p[2]):+.0f}°) {c:.2f}" for c, n, p in scored)
        c, _, p = scored[0]
        self.pose = np.array(p, float); self.prev_pose = self.pose.copy(); self.conf = float(c)
        return self.conf

    def conf_at(self, x, y, th):
        pts = getattr(self, "last_pts", None)
        if pts is None or len(pts) == 0: return None
        return self.matcher._score(x, y, th, pts) / len(pts)

    def localize(self, pts, dt, t_now=None):
        moving = self.pwm != 0
        imu = self.imu
        if imu is not None:
            imu.set_stationary(not moving)                       # 정지 신호로 자이로 바이어스 추적
        if moving:
            d = _rel(self.prev_pose, self.pose) * 0.5
            gdy = imu.delta_yaw(self.t_scan, t_now) if (imu is not None and self.t_scan is not None and t_now is not None) else None
            if gdy is not None:
                d[2] = gdy                                       # ★헤딩은 자이로가 결정(스캔매칭 헤딩드리프트 대체)
            else:
                d[2] = max(-0.15, min(0.15, d[2]))               # 자이로 없으면 기존 상한 예측
            pred = _compose(self.pose, d)
        else:
            pred = self.pose.copy()
        self.t_scan = t_now
        self.prev_pose = self.pose.copy()
        m = np.array(self.matcher.match(tuple(pred), pts), float)
        c = self.matcher._score(m[0], m[1], m[2], pts) / max(1, len(pts))
        gr = imu.rate() if imu is not None else None
        if gr is not None:
            self.yaw_rate = float(gr)                            # ★조향 서보용 요레이트 = 자이로(400Hz) 직접
        if c >= self.a.min_conf:
            dth = _wrap(m[2] - self.pose[2]); dxy = math.hypot(m[0] - self.pose[0], m[1] - self.pose[1])
            if dt > 0:
                if gr is None:
                    self.yaw_rate = 0.6 * self.yaw_rate + 0.4 * (dth / dt)   # 자이로 없을 때만 스캔매칭 Δθ
                self.speed_est = 0.6 * self.speed_est + 0.4 * (dxy / dt)
            self.pose = m; self.lost = 0
        else:
            self.pose = np.array(pred, float); self.lost += 1
            if self.lost % 10 == 0:                          # 1초마다 복구 시도
                lvl = 1 if self.lost < 30 else 2
                p, _ = heading_sweep(self.matcher, tuple(self.pose), pts) if lvl == 1 else \
                    local_search(self.matcher, tuple(self.pose), pts, radius_m=2.0, step_m=0.25, head_step_deg=10.0)
                p = self.matcher.match(tuple(p), pts)
                rc = self.matcher._score(p[0], p[1], p[2], pts) / max(1, len(pts))
                if rc >= max(self.a.min_conf, 0.2):
                    self.pose = np.array(p, float); self.lost = 0; c = rc
        self.conf = float(c)
        if not self.trail or math.hypot(self.trail[-1][0] - self.pose[0], self.trail[-1][1] - self.pose[1]) > 0.05:
            self.trail.append((float(self.pose[0]), float(self.pose[1])))

    # ---------- 계획 ----------
    def plan(self):
        if self.goal is None: return
        p = astar(self.cm, (self.pose[0], self.pose[1]), self.goal)
        self.path = densify(p) if p else []
        self.path_t = time.time()

    # ---------- 제어 (한 스캔) ----------
    def control(self, hits, car, manual=False):
        a = self.a
        if manual:
            ahead = hits[(hits[:, 0] > 0.05) & (np.abs(hits[:, 1]) < a.halfw)]
            self.clear = float(ahead[:, 0].min()) if len(ahead) else float("inf")
            self.mode = "manual"; return
        # 정면 클리어런스 (로봇프레임 x 전방, y 좌)
        ahead = hits[(hits[:, 0] > 0.05) & (np.abs(hits[:, 1]) < a.halfw)]
        self.clear = float(ahead[:, 0].min()) if len(ahead) else float("inf")
        behind = hits[(hits[:, 0] < -0.05) & (np.abs(hits[:, 1]) < a.halfw)]
        self.rear_clear = float((-behind[:, 0]).min()) if len(behind) else float("inf")

        if not self.enabled or self.goal is None or self.lost > 0:
            self.pwm = 0; self.steer = 0
            if self.lost > 0: self.mode = "lost"
            elif self.goal is None and self.mode not in ("arrived",): self.mode = "idle"
            if car: car.drive(0)
            return
        gd = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        if gd < a.goal_tol:
            eh = _wrap(self.goal_th - self.pose[2]) if self.goal_th is not None else 0.0
            if self.goal_th is None or abs(eh) < a.orient_deadband:
                self.mode = "arrived"; self.goal = None; self.goal_th = None; self.path = []; self.lookahead = None
                self.pwm = 0; self.steer = 0
                if car: car.drive(0)
                return
            # ★도착 방향맞춤 — 차형은 제자리회전 불가 → 전진/후진 교대(K턴)로 헤딩 회전
            self.mode = "align"; self.hdg_err = float(eh)
            now = time.time()
            if now >= self.align_switch:
                self.align_fwd = not self.align_fwd; self.align_switch = now + a.align_s
            fwd = self.align_fwd
            want_ccw = eh > 0                                    # 헤딩 늘려야(반시계=좌)?
            # 전진: 좌회전=S− / 우회전=S+ (펌웨어 규약).  후진: 부호 반대(뒤로 가며 코가 반대로 돎)
            steer = (-a.steer_pwm if want_ccw else a.steer_pwm) if fwd else (a.steer_pwm if want_ccw else -a.steer_pwm)
            pwm = a.align_pwm if fwd else -a.align_pwm
            if fwd and self.clear < a.stop_dist: pwm = 0          # 앞 막힘 → 이 위상 스킵(다음 위상서 후진)
            if (not fwd) and self.rear_clear < a.escape_rear_stop: pwm = 0
            self.pwm = pwm; self.steer = steer
            if car:
                car.drive(pwm)
                if steer and pwm != 0: car.steer_pulse(steer, a.pulse_ms)
            return
        if not self.path or time.time() - self.path_t > a.replan:
            self.plan()
        if not self.path:
            self.mode = "noplan"; self.pwm = 0; self.steer = 0
            if car: car.drive(0)
            return
        # 전방 주시점: 가장 가까운 경로점부터 lookahead 거리 이상 떨어진 첫 점
        P = np.asarray(self.path); dd = np.hypot(P[:, 0] - self.pose[0], P[:, 1] - self.pose[1])
        i0 = int(dd.argmin()); tgt = P[-1]
        for i in range(i0, len(P)):
            if dd[i] >= a.lookahead: tgt = P[i]; break
        self.lookahead = (float(tgt[0]), float(tgt[1]))
        e = _wrap(math.atan2(tgt[1] - self.pose[1], tgt[0] - self.pose[0]) - self.pose[2])
        self.hdg_err = float(e)

        # 속도: 장애물·헤딩오차·목표 근접에 따라
        now = time.time()
        if now - self.escape_end > 8.0: self.escape_n = 0          # 한동안 탈출 없었으면 카운터 리셋

        # ★후진 기동 (2026-11-04 재작성). 차형 로봇은 제자리회전 불가 → 뒤로 빼며 반대로 꺾어 코를 목표 쪽으로 돌린다.
        #  예전 버그: 후진 처리가 `clear < stop_dist` 안에 있어서, 몇 cm 물러나 앞이 0.45m 를 넘는 순간 2초를 못 채우고
        #  전진으로 돌아갔다. 그리고 목표가 뒤(헤딩오차 130~166°)여도 후진을 고려하지 않아, 좁은 방에서 전진 원호만
        #  반복하며 헤딩오차가 줄지 않았다(11/04 nav 로그). → 진행 중 후진은 앞이 열려도 계속하고, 목표가 뒤면 먼저 후진.
        def _reverse(kind):
            self.mode = kind
            pwm_ = -a.escape_pwm
            steer_ = a.steer_pwm if e > 0 else -a.steer_pwm        # 후진 땐 조향 반대 → 코끝이 목표 쪽으로 돈다
            self.pwm = pwm_; self.steer = steer_
            if car:
                car.drive(pwm_); car.steer_pulse(steer_, a.pulse_ms)

        if self.rev_kind is not None:
            aligned = abs(e) < math.radians(a.rev_exit_deg)
            done = ((now >= self.rev_min_until and aligned) or self.rear_clear < a.escape_rear_stop
                    or now >= self.rev_max_until)
            if not done:
                _reverse("escape" if self.rev_kind == "escape" else "reverse"); return
            why = ("방향 맞음" if aligned else "뒤 막힘" if self.rear_clear < a.escape_rear_stop else "최대시간")
            print(f"\n[{self.rev_kind} 끝] {why} — 헤딩오차 {math.degrees(e):+.0f}°, 뒤범퍼 {self.rear_clear - a.car_rear:.2f}m", flush=True)
            self.rev_kind = None; self.escape_end = now; self.rev_cool = now + a.escape_cooldown

        if self.clear < a.stop_dist:
            # 앞 막힘 → 뒤가 비면 탈출 후진 (최소 escape_s, 방향이 맞거나 뒤가 막힐 때까지, 최대 rev_max_s)
            if now >= self.rev_cool and self.escape_n < a.escape_max and self.rear_clear > a.escape_rear:
                self.escape_n += 1; self.rev_kind = "escape"
                self.rev_min_until = now + a.escape_s; self.rev_max_until = now + max(a.escape_s, a.rev_max_s)
                print(f"\n[escape #{self.escape_n}] 앞범퍼 {self.clear - a.car_front:.2f}m 막힘, 뒤범퍼 {self.rear_clear - a.car_rear:.2f}m 여유 → 후진", flush=True)
                _reverse("escape"); return
            self.mode = "blocked" if self.escape_n < a.escape_max else "stuck"; pwm = 0
        elif (abs(e) > math.radians(a.rev_turn_deg) and self.rear_clear > a.rev_turn_rear
              and now >= self.rev_cool and gd > a.goal_tol):
            # 목표가 뒤쪽 → 전진 원호 대신 먼저 후진하며 돌린다
            self.rev_kind = "turn"; self.rev_min_until = now + 0.5; self.rev_max_until = now + a.rev_max_s
            print(f"\n[reverse] 목표가 뒤쪽(헤딩오차 {math.degrees(e):+.0f}°), 뒤범퍼 {self.rear_clear - a.car_rear:.2f}m 여유 → 후진하며 회전", flush=True)
            _reverse("reverse"); return
        else:
            self.mode = "drive"
            pwm = a.pwm
            if self.clear < a.slow_dist: pwm = a.pwm_min + (a.pwm - a.pwm_min) * (self.clear - a.stop_dist) / max(1e-3, a.slow_dist - a.stop_dist)
            if abs(e) > math.radians(45): pwm = min(pwm, a.pwm_min + 10)   # 크게 꺾을 땐 천천히
            if gd < 1.0: pwm = min(pwm, a.pwm_min + 15)
            pwm = int(max(a.pwm_min, pwm))
        self.pwm = pwm

        # 조향: 요레이트 서보 (조향각 센서 없음 → 측위 Δθ 로 잰 실제 회전속도를 목표에 맞춤)
        steer = 0
        if pwm > 0 and self.speed_est > 0.03:
            w_d = max(-a.w_max, min(a.w_max, a.k_e * e))
            err = w_d - self.yaw_rate
            if err > a.rate_thr: steer = -a.steer_pwm        # 좌회전 필요 → S − (S + = 오른쪽)
            elif err < -a.rate_thr: steer = +a.steer_pwm
        elif pwm > 0:                                          # 출발 직후(요레이트 아직 없음): 헤딩오차로 직접
            if e > math.radians(20): steer = -a.steer_pwm
            elif e < -math.radians(20): steer = +a.steer_pwm
        self.steer = steer
        if car:
            car.drive(pwm)
            if steer: car.steer_pulse(steer, a.pulse_ms)

    # ---------- 뷰어용 ----------
    def state_json(self):
        with self.lock:
            d = {"pose": [float(v) for v in self.pose], "conf": self.conf, "lost": self.lost, "mode": self.mode,
                 "enabled": self.enabled, "goal": self.goal, "goal_th": (None if self.goal_th is None else float(self.goal_th)), "path": self.path[::3] if self.path else [],
                 "lookahead": self.lookahead, "clear": (None if math.isinf(self.clear) else self.clear),
                 "rear": (None if math.isinf(self.rear_clear) else self.rear_clear), "escape_n": self.escape_n, "car_front": self.a.car_front, "car_rear": self.a.car_rear,
                 "yaw_rate": self.yaw_rate, "speed": self.speed_est, "hdg_err": self.hdg_err,
                 "pwm": self.pwm, "steer": self.steer, "scans": self.n_scan, "hz": self.hz, "ms": self.ms,
                 "trail": self.trail[-2000:], "err": self.err, "off": self.map_m / 2, "res": self.res, "map_m": self.map_m,
                 "t": time.time()}
            # 뷰어용: 현재 스캔(차 기준 last_pts)을 추정 포즈로 맵 좌표에 투영 (11/04) — 벽과 겹치면 측위가 맞다
            if self.last_pts is not None and len(self.last_pts):
                _c, _s = math.cos(self.pose[2]), math.sin(self.pose[2]); _q = self.last_pts[:700]
                d["scan"] = np.column_stack([self.pose[0] + _c * _q[:, 0] - _s * _q[:, 1],
                                             self.pose[1] + _s * _q[:, 0] + _c * _q[:, 1]]).round(3).tolist()
        return json.dumps(d).encode()

    def map_png(self):
        if self._png: return self._png
        g = (255 * (1.0 - self.gm.prob())).astype(np.uint8)
        img = np.stack([g, g, g], -1)
        lethal = self.cm.grid & ~self.wall
        img[lethal] = (255, 205, 205)                    # 치명존(로봇 중심 진입 불가) 연분홍
        from PIL import Image
        buf = io.BytesIO(); Image.fromarray(img[::-1], "RGB").save(buf, "PNG", compress_level=1)
        self._png = buf.getvalue(); return self._png


INDEX_HTML = """<!doctype html><meta charset=utf-8><title>MediScribe NAV</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px monospace}#s{padding:6px 10px;white-space:pre}canvas{display:block;margin:auto;background:#777;cursor:crosshair}</style>
<div id=s>연결 중…</div><div style='padding:0 10px 4px;color:#999'>왼쪽 클릭 = 목표 · 오른쪽 클릭+끌기 = 차 위치/방향 지정 · s 정지 · g 재개 · 초록 점(라이다)이 벽과 겹치면 위치가 맞음</div><canvas id=c></canvas>
<script>
const c=document.getElementById('c'),ctx=c.getContext('2d'),S=document.getElementById('s');let img=new Image(),st=null,k=1;
img.src='/map.png';
async function tick(){try{st=await (await fetch('/state?'+Date.now())).json();
 S.textContent=`[${st.mode}${st.enabled?'':' STOP'}] 신뢰 ${st.conf.toFixed(2)} lost ${st.lost} | PWM ${st.pwm} 조향 ${st.steer} | 앞범퍼 ${st.clear==null?'-':(st.clear-st.car_front).toFixed(2)+'m'} 뒤범퍼 ${st.rear==null?'-':(st.rear-st.car_rear).toFixed(2)+'m'} 헤딩오차 ${(st.hdg_err*180/Math.PI).toFixed(0)}° 요레이트 ${(st.yaw_rate*180/Math.PI).toFixed(0)}°/s v ${st.speed.toFixed(2)} | ${st.hz.toFixed(1)}Hz ${st.ms.toFixed(0)}ms\\n클릭=목표  (${(st.pose[0]-st.off).toFixed(2)}, ${(st.pose[1]-st.off).toFixed(2)}, ${(st.pose[2]*180/Math.PI).toFixed(0)}°)`;
 draw();}catch(e){S.textContent='연결 끊김: '+e}}
function draw(){if(!img.complete)return;const sc=Math.min((innerWidth-4)/img.width,(innerHeight-60)/img.height);c.width=img.width*sc;c.height=img.height*sc;k=sc/st.res;const H=c.height;
 ctx.drawImage(img,0,0,c.width,c.height);
 ctx.strokeStyle='#1e90ff';ctx.lineWidth=2;ctx.beginPath();st.trail.forEach((p,i)=>{i?ctx.lineTo(p[0]*k,H-p[1]*k):ctx.moveTo(p[0]*k,H-p[1]*k)});ctx.stroke();
 ctx.strokeStyle='#2c2';ctx.lineWidth=3;ctx.beginPath();st.path.forEach((p,i)=>{i?ctx.lineTo(p[0]*k,H-p[1]*k):ctx.moveTo(p[0]*k,H-p[1]*k)});ctx.stroke();
 if(st.goal){ctx.fillStyle='#fd0';ctx.beginPath();ctx.arc(st.goal[0]*k,H-st.goal[1]*k,7,0,7);ctx.fill();}
 if(st.lookahead){ctx.fillStyle='#0ff';ctx.beginPath();ctx.arc(st.lookahead[0]*k,H-st.lookahead[1]*k,4,0,7);ctx.fill();}
 if(st.scan){ctx.fillStyle='#3f3';st.scan.forEach(p=>ctx.fillRect(p[0]*k-1.5,H-p[1]*k-1.5,3,3));}
 const x=st.pose[0]*k,y=H-st.pose[1]*k;ctx.fillStyle='#f33';ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.fill();
 ctx.strokeStyle='#fa0';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+Math.cos(st.pose[2])*18,y-Math.sin(st.pose[2])*18);ctx.stroke();}
c.onclick=ev=>{if(!st)return;const x=ev.offsetX/k,y=(c.height-ev.offsetY)/k;fetch(`/goal?x=${x}&y=${y}`,{method:'POST'});};
// 오른쪽 버튼을 누른 채 끌기 = 차 위치 수동 지정(누른 곳=위치, 끈 방향=차가 보는 방향). 짧게 누르면 방향은 지금 그대로 (11/04)
let dn=null;c.oncontextmenu=e=>e.preventDefault();c.onmousedown=e=>{if(e.button==2)dn=[e.offsetX,e.offsetY];};
c.onmouseup=e=>{if(e.button!=2||!dn||!st)return;const x=dn[0]/k,y=(c.height-dn[1])/k,dx=e.offsetX-dn[0],dy=dn[1]-e.offsetY;const deg=Math.hypot(dx,dy)>8?Math.atan2(dy,dx)*180/Math.PI:st.pose[2]*180/Math.PI;fetch(`/init?x=${x}&y=${y}&deg=${deg}`,{method:'POST'});dn=null;};
document.onkeydown=ev=>{if(ev.key=='s')fetch('/stop',{method:'POST'});if(ev.key=='g')fetch('/go',{method:'POST'});};
setInterval(tick,300);tick();
</script>"""


def start_http(nav, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, body, ctype):
            self.send_response(200); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            p = urlparse(self.path).path
            if p == "/state": self._send(nav.state_json(), "application/json")
            elif p == "/conf":
                q = parse_qs(urlparse(self.path).query); x = float(q["x"][0]); y = float(q["y"][0]); th = math.radians(float(q["deg"][0]))
                with nav.lock:
                    c = nav.conf_at(x, y, th); m = nav.matcher.match((x, y, th), nav.last_pts, coarse=WIDE_COARSE) if nav.last_pts is not None else None
                    cm_ = nav.conf_at(*m) if m else None
                self._send(json.dumps({"conf_at": c, "refined": (list(m) if m else None), "conf_refined": cm_}).encode(), "application/json")
            elif p == "/pts":
                with nav.lock: pts = None if nav.last_pts is None else nav.last_pts[::2]
                self._send(json.dumps([] if pts is None else pts.round(3).tolist()).encode(), "application/json")
            elif p == "/map.png": self._send(nav.map_png(), "image/png")
            elif p == "/scan":
                # 로봇프레임(현재 config 오프셋 적용) 8방향 최근접 거리 — 0°=차 정면이어야 함
                with nav.lock: pts = None if nav.last_pts is None else nav.last_pts.copy()
                out = {}
                if pts is not None and len(pts):
                    q = parse_qs(urlparse(self.path).query); sec = float(q.get("sector", ["45"])[0])
                    ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])); dist = np.hypot(pts[:, 0], pts[:, 1])
                    k = -180.0
                    while k < 180:
                        m = (((ang - k + 180) % 360) - 180 > -sec / 2) & (((ang - k + 180) % 360) - 180 <= sec / 2)
                        out[str(int(k))] = (round(float(dist[m].min()), 2) if m.any() else None); k += sec
                    i = int(dist.argmin()); out["nearest"] = {"deg": round(float(ang[i])), "m": round(float(dist[i]), 2)}
                    # 정면 원뿔(±40°, 0.5~2.5m) 안 최근접점의 정확한 각도 — 사람이 정면에 서 있을 때 오프셋 잔차
                    f = (np.abs(ang) < 40) & (dist > 0.5) & (dist < 2.5)
                    if f.any():
                        j = int(np.argmin(np.where(f, dist, 1e9))); out["front_obj"] = {"deg": round(float(ang[j]), 1), "m": round(float(dist[j]), 2)}
                self._send(json.dumps(out).encode(), "application/json")
            elif p in ("/", "/index.html"): self._send(INDEX_HTML.encode(), "text/html; charset=utf-8")
            else: self.send_response(404); self.end_headers()
        def do_POST(self):
            u = urlparse(self.path); q = parse_qs(u.query); p = u.path
            with nav.lock:
                if p == "/goal":
                    nav.goal = (float(q["x"][0]), float(q["y"][0]))
                    nav.goal_th = math.radians(float(q["deg"][0])) if "deg" in q else None   # 드래그 방향 있으면 도착방향
                    nav.path = []; nav.mode = "drive"; nav.enabled = True
                elif p == "/stop": nav.enabled = False
                elif p == "/go": nav.enabled = True
                elif p == "/init":
                    nav.pose = np.array([float(q["x"][0]), float(q["y"][0]), math.radians(float(q.get("deg", ["0"])[0]))])
                    nav.prev_pose = nav.pose.copy(); nav.lost = 0; nav.need_init = False   # ★수동 지정: 준 방향 그대로(자동 sweep 안 함)
                    print(f"수동 위치설정: ({nav.pose[0]-nav.map_m/2:+.2f},{nav.pose[1]-nav.map_m/2:+.2f},{math.degrees(nav.pose[2]):+.0f}°)", flush=True)
                else: self.send_response(404); self.end_headers(); return
            self._send(b"OK", "text/plain")
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True); ap.add_argument("--port", default=None); ap.add_argument("--lidar", default=None)
    ap.add_argument("--init", type=float, nargs=3, default=None, metavar=("X", "Y", "DEG"), help="초기 포즈(맵 m, deg). 기본 맵 중앙(매핑 시작점) 0°")
    ap.add_argument("--init-radius", type=float, default=1.5, help="시작 시 위치 탐색 반경 [m]")
    ap.add_argument("--pwm", type=int, default=130); ap.add_argument("--pwm-min", type=int, default=100)
    ap.add_argument("--steer-pwm", type=int, default=255); ap.add_argument("--pulse-ms", type=int, default=80)
    ap.add_argument("--lookahead", type=float, default=0.6); ap.add_argument("--goal-tol", type=float, default=0.35)
    ap.add_argument("--stop-dist", type=float, default=None, help="정면 정지 [m, 라이다 기준] (기본 = 앞범퍼 + 0.20)")
    ap.add_argument("--slow-dist", type=float, default=None, help="정면 감속 시작 [m, 라이다 기준] (기본 = 앞범퍼 + 0.70)")
    ap.add_argument("--halfw", type=float, default=None, help="앞뒤 판정 반폭 [m] (기본 = 차폭/2 + 0.05)")
    ap.add_argument("--radius", type=float, default=None, help="코스트맵 하드블록 반지름 [m] (기본 = 차폭/2 + 0.08)"); ap.add_argument("--inflation", type=float, default=0.25)
    ap.add_argument("--k-e", type=float, default=1.5, help="헤딩오차→목표 요레이트 이득 [1/s]")
    ap.add_argument("--w-max", type=float, default=0.8, help="목표 요레이트 상한 [rad/s]"); ap.add_argument("--rate-thr", type=float, default=0.25, help="요레이트 오차 문턱 [rad/s]")
    ap.add_argument("--replan", type=float, default=2.0); ap.add_argument("--min-conf", type=float, default=0.18)
    ap.add_argument("--min-range", type=float, default=0.15); ap.add_argument("--decim", type=int, default=MATCH_DECIM)
    ap.add_argument("--http", type=int, default=8766); ap.add_argument("--no-motor", action="store_true")
    ap.add_argument("--max-manual", type=int, default=140, help="조이스틱 수동(RB) 최대 PWM")
    ap.add_argument("--escape-pwm", type=int, default=120, help="막힘 탈출 후진 PWM")
    ap.add_argument("--escape-s", type=float, default=2.0, help="탈출 후진 시간 [s]")
    ap.add_argument("--escape-rear", type=float, default=None, help="탈출 후진 시작에 필요한 후방 거리 [m, 라이다 기준] (기본 = 뒤범퍼 + 0.40)")
    ap.add_argument("--escape-rear-stop", type=float, default=None, help="후진 중 이 미만이면 즉시 중단 [m, 라이다 기준] (기본 = 뒤범퍼 + 0.20)")
    ap.add_argument("--escape-cooldown", type=float, default=1.0, help="탈출 사이 최소 간격 [s]")
    ap.add_argument("--escape-max", type=int, default=6, help="연속 탈출 최대 횟수(초과=stuck 정지)")
    ap.add_argument("--rev-turn-deg", type=float, default=100.0, help="목표가 이 각도보다 뒤에 있으면 먼저 후진하며 회전")
    ap.add_argument("--rev-exit-deg", type=float, default=50.0, help="후진 중 헤딩오차가 이 안으로 들어오면 전진으로")
    ap.add_argument("--rev-turn-rear", type=float, default=None, help="후진 회전 시작에 필요한 후방 거리 [m, 라이다 기준] (기본 = 뒤범퍼 + 0.40)")
    ap.add_argument("--rev-max-s", type=float, default=6.0, help="후진 기동 최대 시간 [s]")
    ap.add_argument("--align-pwm", type=int, default=110, help="도착 방향맞춤(K턴) 구동 PWM")
    ap.add_argument("--align-s", type=float, default=1.5, help="K턴 전진/후진 위상 길이 [s]")
    ap.add_argument("--orient-deadband", type=float, default=math.radians(12), help="도착 방향 허용오차 [rad]")
    # 차체 치수 (2026-11-04 사용자 실측). 거리는 전부 라이다에서 재므로, 범퍼까지의 거리를 더해 기준을 만든다.
    #  예전 기본값(정지 0.45 / 후진정지 0.35 / 후진시작 0.60 / 판정폭 ±0.20 / 경로반경 0.22)은 라이다가 차 가운데 있다고
    #  가정한 값이었다. 실제로는 앞범퍼 0.30·뒤범퍼 0.70 이라, 후진정지 0.35 는 이미 뒤범퍼가 장애물을 지난 뒤였다.
    ap.add_argument("--car-front", type=float, default=0.30, help="라이다 → 앞범퍼 [m]")
    ap.add_argument("--car-rear", type=float, default=0.70, help="라이다 → 뒤범퍼 [m]")
    ap.add_argument("--car-halfw", type=float, default=0.225, help="차폭/2 [m] (차폭 45cm)")
    a = ap.parse_args()
    def _dflt(name, val):
        if getattr(a, name) is None: setattr(a, name, round(val, 3))
    _dflt("stop_dist", a.car_front + 0.20); _dflt("slow_dist", a.car_front + 0.70)
    _dflt("halfw", a.car_halfw + 0.05);     _dflt("radius", a.car_halfw + 0.08)
    _dflt("escape_rear", a.car_rear + 0.40); _dflt("escape_rear_stop", a.car_rear + 0.20)
    _dflt("rev_turn_rear", a.car_rear + 0.40)
    print(f"차체: 앞 {a.car_front:.2f} 뒤 {a.car_rear:.2f} 폭/2 {a.car_halfw:.3f} m (라이다 기준) → "
          f"정지 {a.stop_dist} 감속 {a.slow_dist} 판정폭 ±{a.halfw} 경로반경 {a.radius} | "
          f"후진 시작 {a.escape_rear}/{a.rev_turn_rear} 후진 정지 {a.escape_rear_stop}", flush=True)
    LAST_POSE = os.path.join(ROOT, "out", "last_pose.json")
    if a.init is None:
        lp = load_last_pose(LAST_POSE, os.path.basename(a.map))
        if lp:
            a.init = (lp[0], lp[1], math.degrees(lp[2]))          # ★기억된 마지막 위치(매핑 끝/직전 주행 끝)에서 시작
            print(f"시작 위치: 기억된 마지막 위치 ({a.init[0]:.2f},{a.init[1]:.2f},{a.init[2]:.0f}°)  [--init 로 덮어쓰기 가능]")
        else:
            _g = OccupancyGridMap.load(a.map); a.init = (_g.W * _g.res / 2, _g.H * _g.res / 2, 0.0)   # 기본: 매핑 시작점
            print("시작 위치: 기억 없음 → 맵 중앙(매핑 시작점) 가정")
    nav = Nav(argparse.Namespace(**vars(a)))
    nav.need_init = True

    js = find_js(); fd = os.open(js, os.O_RDONLY | os.O_NONBLOCK) if js else None
    car = None
    if not a.no_motor:
        from medinav.hal.arduino_v3 import ArduinoV3
        car = ArduinoV3(a.port)
        if not car.ping(): sys.exit("아두이노 v3 응답 없음")
        print(f"아두이노 {car.port} OK")
    imu, imu_msg = open_iio_gyro()
    print(f"자이로(헤딩융합): {imu_msg}")
    nav.imu = imu
    lidar = STL27L(a.lidar)
    if lidar.wait_scan(3.0) is None: sys.exit("라이다 스캔 없음")
    print(f"라이다 {lidar.port} OK | 지도 {a.map} ({nav.map_m:.0f}m, {nav.res}m/셀) | 치명존 반지름 {a.radius}m")
    if a.http:
        start_http(nav, a.http); import socket
        print(f"뷰어: 브라우저 http://{socket.gethostname()}.local:{a.http}  (젯슨에서는 http://localhost:{a.http})")
    log = open(a.map.replace(".npz", "") + time.strftime("_nav_%Y%m%d_%H%M%S.log"), "a")

    t_prev = None; last_status = ""; t_log = 0
    joy = JoyThread(fd, car, nav, a.max_manual)   # 50Hz 별도 스레드 — 스캔루프(10Hz)와 분리해 즉각 반응
    try:
        for scan in lidar.iter_scans():
            t0 = time.time()
            if joy.quit: print("\n조이스틱 Back → 종료"); raise KeyboardInterrupt
            manual = joy.manual                                     # 수동 구동은 JoyThread(50Hz)가 담당
            angs, rngs = scan_to_rays(scan)
            keep = rngs >= a.min_range; angs, rngs = angs[keep], rngs[keep]
            if len(angs) < 50: continue
            m = rngs < MAX_RANGE - 1e-3
            hits = np.column_stack([rngs[m] * np.cos(angs[m]), rngs[m] * np.sin(angs[m])])
            pts = hits[::a.decim]
            with nav.lock:
                nav.last_pts = pts
                if nav.need_init:
                    c = nav.init_localize(pts); nav.need_init = False; nav.lost = 0 if c >= a.min_conf else 1
                    print(f"초기 측위 후보: {nav.init_report}  → 채택 신뢰 {c:.2f}")
                else:
                    nav.localize(pts, (t0 - t_prev) if t_prev else 0.1, t0)
                nav.control(hits, car, manual=manual)
                nav.n_scan += 1; nav.ms = (time.time() - t0) * 1000
                if t_prev: nav.hz = 0.8 * nav.hz + 0.2 / max(1e-3, t0 - t_prev)
                st = (f"\r[{nav.mode}{'' if nav.enabled else ' STOP'}] 신뢰 {nav.conf:.2f} lost {nav.lost} | PWM {nav.pwm:4d} 조향 {nav.steer:+4d} | "
                      f"정면 {nav.clear:4.2f}m 헤딩오차 {math.degrees(nav.hdg_err):+4.0f}° 요레이트 {math.degrees(nav.yaw_rate):+4.0f}°/s | "
                      f"pose ({nav.pose[0]-nav.map_m/2:+5.2f},{nav.pose[1]-nav.map_m/2:+5.2f},{math.degrees(nav.pose[2]):+4.0f}°) {nav.hz:4.1f}Hz {nav.ms:3.0f}ms   ")
            t_prev = t0
            if st != last_status: print(st, end="", flush=True); last_status = st
            if t0 - t_log > 2.0:
                t_log = t0; log.write(time.strftime("%H:%M:%S ") + st.strip() + "\n"); log.flush()
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        try:                                                  # 측위가 확실할 때만 마지막 위치 갱신 (틀린 위치 기억 방지)
            if nav.lost == 0 and loc_ok_to_save(nav.conf):
                save_last_pose(LAST_POSE, float(nav.pose[0]), float(nav.pose[1]), float(nav.pose[2]), os.path.basename(a.map))
                print(f"마지막 위치 기억 ({nav.pose[0]-nav.map_m/2:+.2f},{nav.pose[1]-nav.map_m/2:+.2f},{math.degrees(nav.pose[2]):+.0f}°)")
            else:
                print("측위 불확실(lost/저신뢰) → 마지막 위치 저장 안 함")
        except Exception as e:
            print("last_pose 저장 실패:", e)
        if car:
            try: car.close()
            except Exception: pass
        lidar.close(); log.close()
        if fd is not None: os.close(fd)


if __name__ == "__main__":
    main()
