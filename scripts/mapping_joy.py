#!/usr/bin/env python3
"""
조이스틱으로 몰면서 지도를 만든다 — 엔코더 없는 애커만 차량 + STL-27L 라이다 + D435i 자이로.

오도메트리를 바퀴 대신 라이다로 만든다:
  매 스캔(10 Hz)을 '지금까지 만든 지도'에 정합(scan-to-map, 탐색창 ±0.18 m/±11°)해 이동량을 얻는다.
  회전은 자이로로 예측하고 검증한다(정합 초기값 + 급회전 추종).
  정합 신뢰도가 낮은 스캔은 지도에 넣지 않고, 연속으로 놓치면 주변을 넓게 다시 찾아 복구한다.
그 포즈로 키프레임(0.5 m/25° 마다) 그래프를 쌓고 점유격자를 갱신한다.
루프 닫힘은 기본으로 끈다 — 엔코더가 없으면 잘못된 루프 채택이 지도를 망치는 일이 더 잦았다(--loops 로 켬).

조작:
  RB 꾹(데드맨) + RT / LT   = 전진 / 후진 (트리거 깊이 × 최대 PWM)
  왼쪽 스틱 좌우            = 조향 펄스 (오른쪽 = S +)
  Y / A                     = 최대 PWM +20 / −20      Start = 최대 PWM 초기화
  B 또는 Back               = 정지 → 지도 저장 → 종료
저장: out/joymap_<시각>.npz (+.png: 궤적 파랑·현재 위치 빨강) — --save-every 초마다, 그리고 종료 시
      out/joymap_<시각>.log  상태 로그(5초마다)
실시간 보기: 브라우저 http://<젯슨>:8765  — 지도, 궤적, 현재 위치·방향, 현재 라이다 점(초록)

실행 (저장소 루트에서):
  python3 scripts/mapping_joy.py                 # 실차
  python3 scripts/mapping_joy.py --no-motor      # 모터 없이 라이다+SLAM 만 (차를 손으로 밀어 시험)

⚠️ 천천히 몰 것. 라이다 정합은 스캔 사이 ±11° 안에서만 따라간다(10 Hz 면 110°/s).
   특징 없는 긴 복도는 진행 방향 추정이 흐려진다 → 문·기둥이 보이는 쪽으로.
"""
import signal as _sig
_sig.signal(_sig.SIGINT, _sig.default_int_handler)      # &로 띄우면 SIGINT 가 무시로 상속됨 → 복구
def _term(*_): raise KeyboardInterrupt
_sig.signal(_sig.SIGTERM, _term)                         # stop_*.sh 의 TERM 도 정상종료(finally 저장) 경로로
import os, sys, io, time, math, glob, json, select, struct, argparse, threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

os.environ.setdefault("MPLBACKEND", "Agg")
try:
    import matplotlib.image  # noqa: F401,E402  선로딩 — 주행 중 첫 자동저장 import 스파이크 제거
except Exception:
    pass
from medinav.hal.lidar_stl27l import STL27L          # noqa: E402
from medinav.nav.relocalize import local_search     # noqa: E402  상실 시 지역 재탐색
from medinav.hal.lidar import scan_to_rays            # noqa: E402  각도 규약 정본(FLIP·오프셋)
from medinav.nav.graph_slam import GraphSLAM          # noqa: E402
from medinav.hal.imu_iio import open_iio_gyro       # noqa: E402  D435i 자이로(iio) — 헤딩 융합
from medinav.config import RES, MAX_RANGE, MATCH_DECIM  # noqa: E402
from medinav.core.se2 import rel as _rel, compose as _compose  # noqa: E402
from medinav.io.lastpose import save_last_pose                 # noqa: E402  마지막 위치 기억 → nav_drive 자동 시작

# ---- 조이스틱 (teleop_joy.py 와 동일 매핑: F710 XInput/xpad) ----
EV = struct.Struct("IhBB")
AX_LX, AX_LY, AX_LT, AX_RX, AX_RY, AX_RT = 0, 1, 2, 3, 4, 5
BT_A, BT_B, BT_X, BT_Y, BT_LB, BT_RB, BT_BACK, BT_START = 0, 1, 2, 3, 4, 5, 6, 7


def find_js():
    hits = [h for h in sorted(glob.glob("/dev/input/by-id/*-joystick")) if "event" not in h] \
        or sorted(glob.glob("/dev/input/js*"))
    return hits[0] if hits else None


class Shared:
    def __init__(self, pose):
        self.lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.running = True
        self.pose = tuple(pose)
        self.trail = []            # 궤적 (x, y) — 저장 PNG 에 그림
        self.scan_w = []           # 현재 스캔 점(맵 좌표) — 뷰어 겹쳐 그리기용 (11/04)
        self.n_scan = 0
        self.match_ms = 0.0        # 최근 스캔 처리 시간(정합+SLAM)
        self.conf = 0.0            # 정합 신뢰도(점당 평균 필드값, 0~1)
        self.n_skip = 0            # 신뢰도 낮아 지도에 안 넣은 스캔 수
        self.lost_n = 0            # 연속 저신뢰 스캔 수 (10 이상 = LOST)
        self.n_recover = 0         # 재탐색 복구 횟수
        self.min_conf = 0.12
        self.scan_hz = 0.0
        self.last_scan_t = 0.0
        self.lidar_err = ""
        self.pwm = 0; self.steer = 0; self.vmax = 0; self.deadman = False
        self.min_range = 0.15
        self._png = b""; self._png_t = 0.0


def lidar_thread(sh, lidar, slam, decim, imu=None):
    """라이다 오도메(매 스캔 정합) → GraphSLAM. 바퀴 오도메 자리를 정합 포즈가 대체."""
    track = np.array(slam.pose, float)
    prev_track = track.copy()
    t_prev = None
    try:
        for scan in lidar.iter_scans():
            if not sh.running:
                break
            t0 = time.time()
            angs, rngs = scan_to_rays(scan)
            keep = rngs >= sh.min_range                  # 차체·거치대 반사(15cm 이내) 제거
            angs, rngs = angs[keep], rngs[keep]
            if len(angs) < 50:
                continue
            hits = slam._hits(angs, rngs)
            pts = hits[::decim]
            conf = 1.0
            with sh.map_lock:
                if slam.keyframes and len(pts) > 10:
                    # 1) 등속 예측: 직전 스캔 간 이동량을 한 번 더 적용한 자리에서 탐색 시작 (회전 중 추종성)
                    with sh.lock:
                        moving = (sh.pwm != 0) or (sh.steer != 0)
                    if imu is not None:
                        imu.set_stationary(not moving)                # 정지 신호로 자이로 바이어스 추적
                    if moving:
                        d = _rel(prev_track, track) * 0.5            # 등속 예측은 절반만 (되먹임 방지)
                        with sh.lock:
                            cmd = sh.pwm
                        if cmd == 0 or cmd * d[0] < 0:                # ★조향만 하거나 전진↔후진 전환 → 직전 이동 외삽 금지
                            d[0] = 0.0; d[1] = 0.0
                        gdy = imu.delta_yaw(t_prev, t0) if (imu is not None and t_prev is not None) else None
                        if gdy is not None:
                            d[2] = gdy                                # ★헤딩은 자이로가 결정(스캔매칭 헤딩드리프트 대체)
                        else:
                            d[2] = max(-0.15, min(0.15, d[2]))        # 자이로 없으면 기존 ±8.6°/스캔 상한
                        pred = _compose(track, d)
                    else:
                        pred = track.copy()                           # 정지 명령 중엔 예측 없음 → 제자리 정합
                    prev_track = track.copy()
                    # 2) 라이다 오도메: scan-to-map 정합
                    m = np.array(slam.matcher.match(tuple(pred), pts), float)
                    conf = slam.matcher._score(m[0], m[1], m[2], pts) / max(1, len(pts))
                    if conf >= sh.min_conf:
                        track = m
                        pose = slam.process(track, angs, rngs)   # 3) 키프레임·맵 누적
                        track = np.array(pose, float)
                        with sh.lock: sh.lost_n = 0
                    else:                                        # 신뢰도 낮음: 예측만 믿고 지도엔 안 넣음
                        track = pred; pose = track
                        with sh.lock:
                            sh.n_skip += 1; sh.lost_n += 1; ln = sh.lost_n
                        if ln % 10 == 0:                         # ★상실 1초마다 지역 재탐색(자이로 헤딩 기준 ±1.2m)
                            try:
                                cand, _ = local_search(slam.matcher, tuple(track), pts,
                                                       radius_m=1.2, step_m=0.2, head_step_deg=10.0)
                                cand = np.array(slam.matcher.match(tuple(cand), pts), float)
                                cc = slam.matcher._score(cand[0], cand[1], cand[2], pts) / max(1, len(pts))
                                if cc >= max(sh.min_conf, 0.20):
                                    track = cand; pose = track; conf = cc
                                    with sh.lock: sh.lost_n = 0; sh.n_recover += 1
                            except Exception:
                                pass
                else:
                    prev_track = track.copy()
                    pose = slam.process(track, angs, rngs)
                    track = np.array(pose, float)
            dt = time.time() - t0
            # 뷰어용: 이번 스캔을 맵 좌표로 (차 기준 hits 를 현재 포즈로 변환) — 벽과 겹치는지가 곧 정합 품질
            _c, _s = math.cos(pose[2]), math.sin(pose[2]); _q = hits[::3][:700]
            scan_w = np.column_stack([pose[0] + _c * _q[:, 0] - _s * _q[:, 1],
                                      pose[1] + _s * _q[:, 0] + _c * _q[:, 1]]).round(3).tolist()
            with sh.lock:
                sh.pose = (float(pose[0]), float(pose[1]), float(pose[2]))
                sh.scan_w = scan_w
                sh.n_scan += 1
                sh.match_ms = dt * 1000.0
                sh.conf = float(conf)
                if t_prev is not None:
                    sh.scan_hz = 0.8 * sh.scan_hz + 0.2 * (1.0 / max(1e-3, t0 - t_prev))
                sh.last_scan_t = t0
                if not sh.trail or math.hypot(sh.trail[-1][0] - pose[0], sh.trail[-1][1] - pose[1]) > 0.05:
                    sh.trail.append((float(pose[0]), float(pose[1])))
            t_prev = t0
    except Exception as e:
        with sh.lock:
            sh.lidar_err = f"{type(e).__name__}: {e}"
            sh.running = False


INDEX_HTML = """<!doctype html><meta charset=utf-8><title>MediScribe SLAM</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px monospace}#s{padding:6px 10px}canvas{display:block;margin:auto;background:#777}</style>
<div id=s>연결 중…</div><canvas id=c></canvas>
<script>
const c=document.getElementById('c'),ctx=c.getContext('2d'),S=document.getElementById('s');let img=new Image(),st=null;
async function tick(){try{st=await (await fetch('/state?'+Date.now())).json();
 S.textContent=`KF ${st.kf}  루프 ${st.loops}  스캔 ${st.scans} ${st.hz.toFixed(1)}Hz ${st.ms.toFixed(0)}ms  |  PWM ${st.pwm} (max ${st.vmax}) 조향 ${st.steer} ${st.deadman?'●RB':'○RB'}  |  pose (${(st.pose[0]-st.off).toFixed(2)}, ${(st.pose[1]-st.off).toFixed(2)}, ${(st.pose[2]*180/Math.PI).toFixed(0)}°)`;
 img.src='/map.png?'+Date.now();}catch(e){S.textContent='연결 끊김: '+e}}
img.onload=()=>{const sc=Math.min((innerWidth-4)/img.width,(innerHeight-40)/img.height);c.width=img.width*sc;c.height=img.height*sc;
 ctx.drawImage(img,0,0,c.width,c.height);if(!st)return;const k=sc/st.res,H=c.height;
 ctx.strokeStyle='#1e90ff';ctx.lineWidth=2;ctx.beginPath();st.trail.forEach((p,i)=>{const x=p[0]*k,y=H-p[1]*k;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();
 if(st.scan){ctx.fillStyle='#3f3';st.scan.forEach(p=>ctx.fillRect(p[0]*k-1.5,H-p[1]*k-1.5,3,3));}
 const x=st.pose[0]*k,y=H-st.pose[1]*k;ctx.fillStyle='#f33';ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.fill();
 ctx.strokeStyle='#fa0';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+Math.cos(st.pose[2])*18,y-Math.sin(st.pose[2])*18);ctx.stroke();};
setInterval(tick,300);tick();
</script>"""


def make_http_handler(sh, slam, off):
    def state_json():
        with sh.lock:
            d = {"pose": list(sh.pose), "scans": sh.n_scan, "hz": sh.scan_hz, "ms": sh.match_ms,
                 "pwm": sh.pwm, "steer": sh.steer, "vmax": sh.vmax, "deadman": sh.deadman,
                 "conf": sh.conf, "skip": sh.n_skip, "lost_n": sh.lost_n, "recover": sh.n_recover,
                 "trail": sh.trail[-3000:], "err": sh.lidar_err, "t": time.time(), "scan": sh.scan_w}
        with sh.map_lock:
            d["kf"] = len(slam.keyframes); d["loops"] = slam.n_loops
        d["off"] = off; d["res"] = slam.res; d["map_m"] = slam.gm.W * slam.res
        return json.dumps(d).encode()

    def map_png():
        """현재 점유확률 → 8bit 회색 PNG (벽 검정, 빈칸 흰색, 미지 회색). 0.25s 캐시. 행0 = y=0 (뷰어가 origin=lower 로 그림)."""
        now = time.time()
        with sh.lock:
            if now - sh._png_t < 0.25 and sh._png:
                return sh._png
        with sh.map_lock:
            g = (255 * (1.0 - slam.gm.prob())).astype(np.uint8)
        from PIL import Image
        buf = io.BytesIO(); Image.fromarray(g[::-1], "L").save(buf, "PNG", compress_level=1)
        with sh.lock:
            sh._png = buf.getvalue(); sh._png_t = now
        return sh._png

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/state":
                body, ctype = state_json(), "application/json"
            elif path == "/map.png":
                body, ctype = map_png(), "image/png"
            elif path in ("/", "/index.html"):
                body, ctype = INDEX_HTML.encode(), "text/html; charset=utf-8"
            else:
                self.send_response(404); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(body)
    return H


def start_http(sh, slam, off, port):
    srv = ThreadingHTTPServer(("0.0.0.0", port), make_http_handler(sh, slam, off))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def save_map(sh, slam, out_npz):
    """npz(GraphSLAM.save) + PNG(궤적 파랑, 현재 위치 빨강). matplotlib→PIL→PPM 순으로 폴백."""
    with sh.map_lock:
        slam.save(out_npz)
        prob = slam.gm.prob()
        kf, loops = len(slam.keyframes), slam.n_loops
    with sh.lock:
        trail = list(sh.trail); px, py, _ = sh.pose
    g = (255 * (1.0 - prob)).astype(np.uint8)              # 벽=검정, 빈칸=흰색, 미지=회색
    img = np.stack([g, g, g], axis=-1)
    H, W = g.shape
    res = slam.res
    for (x, y) in trail:
        c, r = int(x / res), int(y / res)
        if 0 <= r < H and 0 <= c < W:
            img[r, c] = (30, 144, 255)
    c, r = int(px / res), int(py / res)
    img[max(0, r - 2):r + 3, max(0, c - 2):c + 3] = (255, 40, 40)
    img = img[::-1]                                        # origin lower → 이미지 위가 +y
    png = out_npz.replace(".npz", ".png")
    try:
        os.environ.setdefault("MPLBACKEND", "Agg")
        import matplotlib.image as mpimg
        mpimg.imsave(png, img)
    except Exception:
        try:
            from PIL import Image
            Image.fromarray(img).save(png)
        except Exception:
            png = out_npz.replace(".npz", ".ppm")
            with open(png, "wb") as f:
                f.write(f"P6 {W} {H} 255\n".encode()); f.write(img.tobytes())
    return png, kf, loops


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="아두이노 (기본: by-id 자동)")
    ap.add_argument("--lidar", default=None, help="라이다 (기본: by-id 자동)")
    ap.add_argument("--js", default=None)
    ap.add_argument("--max", type=int, default=120, help="최대 구동 PWM (기본 120 — 빠르면 정합 놓침, Y로 +20)")
    ap.add_argument("--min-pwm", type=int, default=90, help="트리거를 조금만 눌러도 이 출력은 보장 (모터 기동 하한)")
    ap.add_argument("--steer-pwm", type=int, default=255, help="조향 출력 — 스틱을 기울이면 무조건 이 값(bang-bang)")
    ap.add_argument("--steer-ms", type=int, default=150, help="조향 펄스 길이 [ms] (스틱 잡고 있으면 계속 재발행)")
    ap.add_argument("--map-size", type=float, default=30.0, help="맵 한 변 [m]")
    ap.add_argument("--decim", type=int, default=MATCH_DECIM, help="정합에 쓰는 점 1/N (기본 config)")
    ap.add_argument("--save-every", type=float, default=30.0, help="자동 저장 주기 [s]")
    ap.add_argument("--no-motor", action="store_true", help="아두이노 없이 라이다+SLAM 만")
    ap.add_argument("--hz", type=float, default=50)
    ap.add_argument("--http", type=int, default=8765, help="실시간 보기 포트 (0=끔)")
    ap.add_argument("--min-range", type=float, default=0.15, help="이 거리[m] 안 반사는 버림 (차체)")
    ap.add_argument("--min-conf", type=float, default=0.12, help="정합 신뢰도(점당 필드값) 이 미만이면 지도에 안 넣음")
    ap.add_argument("--loops", action="store_true", help="루프 닫힘 켬 (기본 끔 — 엔코더 없는 차에선 오탐이 지도를 망침)")
    ap.add_argument("--loop-accept", type=float, default=0.75, help="--loops 일 때 루프 채택 문턱 (엄격하게 — 기본 0.75)")
    ap.add_argument("--loop-gap", type=int, default=20, help="--loops 일 때 루프 후보 최소 키프레임 간격")
    ap.add_argument("--loop-radius", type=float, default=1.0)
    a = ap.parse_args()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(ROOT, "out"); os.makedirs(out_dir, exist_ok=True)
    out_npz = os.path.join(out_dir, f"joymap_{ts}.npz")
    log_path = out_npz.replace(".npz", ".log")
    logf = open(log_path, "a")

    def log(msg):
        line = time.strftime("%H:%M:%S ") + msg
        logf.write(line + "\n"); logf.flush()

    # ---- 장치 ----
    js = a.js or find_js()
    fd = None
    if js:
        fd = os.open(js, os.O_RDONLY | os.O_NONBLOCK)
    elif not a.no_motor:
        sys.exit("조이스틱 없음 (/dev/input/js*). F710 뒷면 스위치 X, 동글, xpad 모듈 확인. 라이다만 돌리려면 --no-motor")

    car = None
    if not a.no_motor:
        from medinav.hal.arduino_v3 import ArduinoV3
        car = ArduinoV3(a.port)
        if not car.ping():
            sys.exit("아두이노 v3 응답 없음 ('?' 배너 없음). 포트/전원 확인")
        print(f"아두이노 {car.port} v3 OK")

    lidar = STL27L(a.lidar)
    first = lidar.wait_scan(3.0)
    if first is None:
        sys.exit(f"라이다 {lidar.port}: 3초 동안 스캔 없음 (패킷 {lidar.packets}, CRC오류 {lidar.crc_errors})")
    print(f"라이다 {lidar.port} OK: 스캔당 {len(first)}점, {lidar.rpm:.0f}rpm")

    slam = GraphSLAM(map_m=a.map_size, res=RES, max_range=MAX_RANGE, match_decim=a.decim,
                     loop_radius=a.loop_radius, loop_min_gap=a.loop_gap,
                     loop_accept=(a.loop_accept if a.loops else 2.0))   # 2.0 = 절대 채택 안 됨(루프 끔)
    sh = Shared(slam.pose)
    sh.min_range = a.min_range; sh.vmax = a.max; sh.min_conf = a.min_conf
    off = slam.OFF
    if a.http:
        start_http(sh, slam, off, a.http)
        import socket
        print(f"실시간 보기: 브라우저 http://{socket.gethostname()}.local:{a.http}  (젯슨에서는 http://localhost:{a.http})")
    imu, imu_msg = open_iio_gyro()
    print(f"자이로(헤딩융합): {imu_msg}")
    threading.Thread(target=lidar_thread, args=(sh, lidar, slam, a.decim, imu), daemon=True).start()

    mode = ("라이다+SLAM 만(--no-motor)" if a.no_motor else f"실차 최대 PWM {a.max}") + (" 루프ON" if a.loops else " 루프OFF")
    print(f"\n★ 조이스틱 그래프 SLAM 매핑  [{mode}]  맵 {a.map_size:.0f}m  저장 {os.path.relpath(out_npz, ROOT)}")
    print("  RB 잡고 RT/LT 전후진, 왼스틱 조향 | Y/A 최대PWM ±20 | B/Back = 저장 후 종료 | Ctrl+C 도 저장")
    print("  천천히 방을 한 바퀴 → 출발점으로 돌아오면 루프가 닫히며 맵이 정렬됩니다.\n")
    log(f"start mode={mode} map={a.map_size} decim={a.decim} lidar={lidar.port} js={js}")

    axes = [0.0] * 8; btn = [0] * 12
    vmax = a.max; period = 1.0 / a.hz
    t_save = t_status = t_log = time.time()
    last_status = ""; stop_req = False
    try:
        while sh.running:
            if fd is not None:
                r, _, _ = select.select([fd], [], [], period)
                if r:
                    try:
                        while True:
                            d = os.read(fd, EV.size)
                            if len(d) < EV.size:
                                break
                            _, val, typ, num = EV.unpack(d)
                            typ &= 0x7F
                            if typ == 2 and num < 8:
                                axes[num] = val / 32767.0
                            elif typ == 1 and num < 12:
                                btn[num] = val
                                if val:
                                    if num == BT_Y: vmax = min(255, vmax + 20)
                                    elif num == BT_A: vmax = max(40, vmax - 20)
                                    elif num == BT_START: vmax = a.max
                                    elif num in (BT_B, BT_BACK): stop_req = True
                    except BlockingIOError:
                        pass
            else:
                time.sleep(period)
            if stop_req:
                print("\nB/Back → 정지 후 저장·종료"); break

            deadman = bool(btn[BT_RB])
            rt = (axes[AX_RT] + 1) / 2; lt = (axes[AX_LT] + 1) / 2
            pwm = int(round((rt - lt) * vmax)) if deadman else 0
            if abs(pwm) < 15: pwm = 0
            elif abs(pwm) < a.min_pwm: pwm = a.min_pwm if pwm > 0 else -a.min_pwm   # 기동 하한
            lx = axes[AX_LX] if deadman else 0.0
            steer = (a.steer_pwm if lx > 0 else -a.steer_pwm) if abs(lx) > 0.15 else 0   # bang-bang 풀파워
            if car:
                car.drive(pwm)
                if steer: car.steer_pulse(steer, a.steer_ms)
            with sh.lock:
                sh.pwm, sh.steer, sh.vmax, sh.deadman = pwm, steer, vmax, deadman

            now = time.time()
            if now - t_status > 0.2:
                t_status = now
                with sh.lock:
                    px, py, th = sh.pose; n = sh.n_scan; ms = sh.match_ms; hz = sh.scan_hz; cf = sh.conf; sk = sh.n_skip; ln = sh.lost_n; rc = sh.n_recover
                with sh.map_lock:
                    kf, loops = len(slam.keyframes), slam.n_loops
                status = (f"\r{'●' if deadman else '○'}RB PWM{pwm:+4d}(max{vmax}) 조향{steer:+4d} | "
                          f"스캔{n:5d} {hz:4.1f}Hz {ms:5.1f}ms 신뢰 {cf:.2f} 스킵 {sk} 복구 {rc}{' ⚠LOST-멈추고 아는 곳으로 천천히' if ln >= 10 else ''} | KF {kf:3d} 루프 {loops:2d} | "
                          f"pose ({px-off:+5.2f},{py-off:+5.2f},{math.degrees(th):+4.0f}°)   ")
                if status != last_status:
                    print(status, end="", flush=True); last_status = status
            if now - t_log > 5.0:
                t_log = now
                with sh.lock:
                    px, py, th = sh.pose; n = sh.n_scan; ms = sh.match_ms; hz = sh.scan_hz
                log(f"scan={n} hz={hz:.1f} ms={ms:.0f} conf={sh.conf:.2f} skip={sh.n_skip} kf={len(slam.keyframes)} loops={slam.n_loops} "
                    f"pose=({px-off:.2f},{py-off:.2f},{math.degrees(th):.0f}) pwm={pwm} steer={steer}")
            if now - t_save > a.save_every and ((pwm == 0 and steer == 0) or now - t_save > 4 * a.save_every):
                t_save = now
                png, kf, loops = save_map(sh, slam, out_npz)
                log(f"autosave {os.path.basename(png)} kf={kf} loops={loops}")
    except KeyboardInterrupt:
        print("\nCtrl+C → 정지 후 저장·종료")
    finally:
        sh.running = False
        if car:
            try: car.close()
            except Exception: pass
        time.sleep(0.3)
        png, kf, loops = save_map(sh, slam, out_npz)
        with sh.lock:
            err = sh.lidar_err; n = sh.n_scan
        try: lidar.close()
        except Exception: pass
        if fd is not None: os.close(fd)
        with open(out_npz.replace(".npz", ".json"), "w") as f:
            json.dump({"keyframes": kf, "loops": loops, "scans": n, "map_size_m": a.map_size,
                       "res": slam.res, "origin_offset_m": off, "trail": sh.trail[-2000:]}, f)
        log(f"end kf={kf} loops={loops} scans={n} err={err!r}")
        logf.close()
        if err: print(f"\n[라이다 스레드 오류] {err}")
        print(f"\n맵 저장: {os.path.relpath(out_npz, ROOT)}  (키프레임 {kf}, 루프 {loops}, 스캔 {n})")
        print(f"그림:    {os.path.relpath(png, ROOT)}")
        try:                                                  # 최종 pose 기억 → nav_drive 가 --init 없이 여기서 시작
            px, py, th = sh.pose
            save_last_pose(os.path.join(ROOT, "out", "last_pose.json"), px, py, th, os.path.basename(out_npz))
            print(f"마지막 위치 기억: ({px-off:+.2f},{py-off:+.2f},{math.degrees(th):+.0f}°) → nav_drive 가 자동으로 여기서 시작")
        except Exception as e:
            print("last_pose 저장 실패:", e)


if __name__ == "__main__":
    main()
