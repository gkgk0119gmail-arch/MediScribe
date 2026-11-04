#!/usr/bin/env python3
"""D435i 자이로 3축 실시간 보기 — 어느 축이 차의 요(좌우 회전)인지, 부호가 맞는지 눈으로 확인한다. 모터 안 켬.

왜 필요한가: 축값을 잘못 디코딩하던 시절(s16 을 32비트로 읽음)에는
축 설정을 제대로 검증할 수 없었다. 카메라 장착 방향에 따라 요축이 x/y/z 중
무엇이 될지 달라진다.

사용 (robot/stack 에서):
  python3 -u scripts/diag_gyro_axes.py            # 90초
  1) 2초 가만히 (바이어스 측정)
  2) 차를 바닥에 둔 채 위에서 봤을 때 **반시계(왼쪽)로 90도** 돌린다
  3) ±90 근처로 바뀐 축이 요축. +면 YAW_SIGN=+1, -면 -1
  r + Enter = 누적각 0 으로 리셋,  Ctrl+C = 종료
결과는 medinav/config.py 의 IMU_GYRO_AXIS('x'/'y'/'z'), IMU_GYRO_SIGN 에 반영한다.
"""
import os, sys, time, struct, select, argparse
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import medinav.hal.imu_iio as m

ap = argparse.ArgumentParser(); ap.add_argument("--secs", type=float, default=90); a = ap.parse_args()
d = m._find_gyro_dir()
if not d:
    sys.exit("gyro_3d 없음 — 카메라를 꽂은 뒤라면: sudo systemctl restart d435i-imu.service")
fmt = [m._chan_fmt(d, c) for c in ("in_anglvel_x", "in_anglvel_y", "in_anglvel_z")]
st = fmt[0][3] // 8
scale = float(open(os.path.join(d, "in_anglvel_scale")).read()) * 57.2958      # → °/s
fd = os.open(m._devnode(d), os.O_RDONLY)
buf = b""


def read(timeout=0.05):
    global buf
    out = []
    r, _, _ = select.select([fd], [], [], timeout)
    if r:
        buf += os.read(fd, m._SAMPLE * 64)
    while len(buf) >= m._SAMPLE:
        rec, buf = buf[:m._SAMPLE], buf[m._SAMPLE:]
        v = [m._decode(rec[i * st:(i + 1) * st], fmt[i]) * scale for i in range(3)]
        out.append((struct.unpack_from("<q", rec, m._TS_OFF)[0] / 1e9, v))
    return out


print("가만히 두세요 — 바이어스 측정 2초...", flush=True)
cal, t0 = [], time.time()
while time.time() - t0 < 2:
    cal += read()
if not cal:
    sys.exit("자이로 데이터가 안 옵니다 — sudo systemctl restart d435i-imu.service 후 다시")
bias = [sum(s[1][i] for s in cal) / len(cal) for i in range(3)]
print(f"바이어스 x{bias[0]:+.2f} y{bias[1]:+.2f} z{bias[2]:+.2f} °/s  ({len(cal) / 2:.0f}Hz)")
print("이제 위에서 봤을 때 반시계(왼쪽)로 90도 돌리세요.   r+Enter 리셋 / Ctrl+C 종료")
print(f"현재 설정: IMU_GYRO_AXIS={'xyz'[m.YAW_AXIS]}  IMU_GYRO_SIGN={m.YAW_SIGN:+.0f}\n")
ang, t_last, t_end, last_print = [0.0] * 3, None, time.time() + a.secs, 0
try:
    while time.time() < t_end:
        for ts, v in read():
            w = [v[i] - bias[i] for i in range(3)]
            if t_last is not None and 0 < ts - t_last < 0.05:
                for i in range(3):
                    if abs(w[i]) > 1.0:
                        ang[i] += w[i] * (ts - t_last)
            t_last = ts
        if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline().strip().lower() == "r":
            ang = [0.0] * 3
        if time.time() - last_print > 0.2:
            last_print = time.time()
            k = max(range(3), key=lambda i: abs(ang[i]))
            sys.stdout.write(f"\r  누적  x {ang[0]:+7.1f}°   y {ang[1]:+7.1f}°   z {ang[2]:+7.1f}°"
                             f"    ← 가장 큰 축 {'xyz'[k]}   ")
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
k = max(range(3), key=lambda i: abs(ang[i]))
print(f"\n\n결과: 요축 후보 = {'xyz'[k]} ({ang[k]:+.1f}°)")
if abs(ang[k]) > 30:
    print(f"반시계 90도를 돌렸다면 → medinav/config.py 에  IMU_GYRO_AXIS = \"{'xyz'[k]}\"   IMU_GYRO_SIGN = {+1.0 if ang[k] > 0 else -1.0:+.1f}")
else:
    print("30도 넘게 돈 축이 없습니다 — 다시 시도하세요.")
