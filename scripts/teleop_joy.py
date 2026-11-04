#!/usr/bin/env python3
"""
조이스틱 텔레옵 + 끊김 진단 기록 — Logitech F710(XInput, /dev/input/js0) → 아두이노(펌웨어 v3).

펌웨어 명령(firmware/PROTOCOL-v3.md): "D <pwm>" 구동, "S <pwm> <ms>" 조향 펄스, "X" 정지.
아두이노는 명령이 ~0.5초 끊기면 스스로 멈추고 "WD STOP" 을 보낸다(워치독). hal/arduino_v3 가 50 Hz 로 재전송한다.

조작:
  RB 꾹(데드맨)  = 잡고 있는 동안만 움직임. 놓으면 즉시 정지
  RT / LT        = 전진 / 후진 (트리거 깊이 × 최대 PWM)
  왼쪽 스틱 좌우 = 조향 펄스
  Y / A          = 최대 PWM +20 / −20,  Start = 초기화
  B 또는 Back    = 정지 후 종료

진단 기록 out/teleop_<시각>.csv (틱마다 한 줄) — 끊김 원인 가르기:
  wd 증가 = 명령 끊김(통신) · ard_ms 감소 = 아두이노 재부팅(전압 강하) ·
  RB·트리거를 잡았는데 pwm 0 = 조이스틱 무선 끊김 · 셋 다 정상 = 모터 드라이버/배선/기구

실행:  python3 scripts/teleop_joy.py [--port /dev/ttyACM0] [--max 120] [--dry]
⚠️ 첫 시험은 바퀴를 띄우고, 물리 비상정지를 손에 두고.
"""
import argparse, glob, os, select, struct, sys, time
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, ROOT)

EV = struct.Struct("IhBB")          # time_ms u32, value s16, type u8, number u8
AX_LX, AX_LY, AX_LT, AX_RX, AX_RY, AX_RT = 0, 1, 2, 3, 4, 5
BT_A, BT_B, BT_X, BT_Y, BT_LB, BT_RB, BT_BACK, BT_START = 0, 1, 2, 3, 4, 5, 6, 7

def find_js():
    hits = [h for h in sorted(glob.glob("/dev/input/by-id/*-joystick")) if "event" not in h] or sorted(glob.glob("/dev/input/js*"))
    return hits[0] if hits else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None); ap.add_argument("--js", default=None)
    ap.add_argument("--max", type=int, default=140, help="최대 구동 PWM (0..255). 9/11 실험: 120 에서 바퀴 돎")
    ap.add_argument("--steer-pwm", type=int, default=255, help="조향 펄스 PWM (왼쪽이 약해 255 권장)")
    ap.add_argument("--dry", action="store_true"); ap.add_argument("--hz", type=float, default=50)
    ap.add_argument("--log", default=None, help="끊김 진단 CSV (기본 out/teleop_<시각>.csv, 'off' 면 안 씀)")
    a = ap.parse_args()

    js = a.js or find_js()
    if not js: sys.exit("조이스틱 없음 (/dev/input/js*). F710 뒷면 스위치 X, 동글 연결, xpad 모듈 확인")
    fd = os.open(js, os.O_RDONLY | os.O_NONBLOCK)
    car = None
    if not a.dry:
        from medinav.hal.arduino_v3 import ArduinoV3
        car = ArduinoV3(a.port)
        if not car.ping(): sys.exit("아두이노 v3 응답 없음 ('?' 에 배너 없음)")
        print(f"아두이노 {car.port} v3 OK")
    print(f"조이스틱 {js} | 최대 PWM {a.max} | {'DRY' if a.dry else '실차'}  — RB 잡고 RT/LT, 왼스틱 조향. B 종료\n")

    # ---- 끊김 진단 기록 (2026-11-04) ----
    # 한 줄 = 한 틱. 원인 가르기:
    #   wd 증가                → 젯슨→아두이노 명령이 0.5s 끊김 (통신)
    #   ard_ms 가 작아짐       → 아두이노 재부팅 (모터 전류에 의한 전압 강하 의심)
    #   RB·트리거 잡았는데 pwm 0 → 조이스틱 무선 끊김 (js_age 도 함께 봄)
    #   셋 다 정상인데 끊김     → 모터 드라이버·배선·기구
    logf = None
    if (a.log or "").lower() != "off":
        os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)
        lp = a.log or os.path.join(ROOT, "out", time.strftime("teleop_%Y%m%d_%H%M%S.csv"))
        logf = open(lp, "w", buffering=1)
        logf.write("t,deadman,pwm,steer,rt,lt,lx,js_age_ms,wd,rx_age_ms,ard_a,ard_b,ard_c,ard_ms,loop_ms\n")
        print(f"진단 기록: {lp}")
    t_js = time.time(); t_prev = time.time(); t0 = time.time()

    axes = [0.0] * 8; btn = [0] * 12; vmax = a.max; period = 1.0 / a.hz
    estop_hold = False; last_status = ""
    try:
        while True:
            r, _, _ = select.select([fd], [], [], period)
            if r:
                try:
                    while True:
                        d = os.read(fd, EV.size)
                        if len(d) < EV.size: break
                        _, val, typ, num = EV.unpack(d); t_js = time.time()
                        typ &= 0x7F                                   # 초기 상태 이벤트 플래그(0x80) 제거
                        if typ == 2 and num < 8: axes[num] = val / 32767.0
                        elif typ == 1 and num < 12:
                            btn[num] = val
                            if val:
                                if num == BT_Y: vmax = min(255, vmax + 20)
                                elif num == BT_A: vmax = max(40, vmax - 20)
                                elif num == BT_START: vmax = a.max
                                elif num in (BT_B, BT_BACK): estop_hold = True
                except BlockingIOError:
                    pass
            if estop_hold:
                print("\n비상정지(E) → 종료"); break
            deadman = bool(btn[BT_RB])
            rt = (axes[AX_RT] + 1) / 2; lt = (axes[AX_LT] + 1) / 2   # 트리거: 안 누르면 -1
            pwm = int(round((rt - lt) * vmax)) if deadman else 0
            if abs(pwm) < 15: pwm = 0
            lx = axes[AX_LX] if deadman else 0.0
            steer = int(round(lx * a.steer_pwm)) if abs(lx) > 0.15 else 0        # 스틱 오른쪽(+) = S +
            if car:
                car.drive(pwm)
                if steer: car.steer_pulse(steer, 100)
            status = f"\r{'●' if deadman else '○'} RB  구동 PWM {pwm:+4d} (max {vmax})  조향 {steer:+4d}   RT {rt:.2f} LT {lt:.2f} LX {lx:+.2f}" + (f"  WD정지 {car.wd_stops}" if car else "") + "   "
            if status != last_status: print(status, end="", flush=True); last_status = status
            if logf:
                now = time.time(); st_ = car.status if car else (0, 0, 0, 0)
                rx_age = (now - car.last_rx) * 1000 if (car and car.last_rx) else -1
                logf.write(f"{now - t0:.3f},{int(deadman)},{pwm},{steer},{rt:.3f},{lt:.3f},{lx:+.3f},"
                           f"{(now - t_js) * 1000:.0f},{car.wd_stops if car else 0},{rx_age:.0f},"
                           f"{st_[0]},{st_[1]},{st_[2]},{st_[3]},{(now - t_prev) * 1000:.1f}\n")
                t_prev = now
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        if car: car.close()
        os.close(fd)
        if logf: logf.close()

if __name__ == "__main__":
    main()
