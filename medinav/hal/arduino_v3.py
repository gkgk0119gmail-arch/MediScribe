"""아두이노 펌웨어 v3 HAL (firmware/PROTOCOL-v3.md). 정수 PWM, 조향은 펄스 개루프.

  drive(pwm)            양쪽 구동 -255..255 (D). 워치독(~0.5s) 때문에 TX 스레드가 50Hz 로 재전송.
  drive_lr(l, r)        바퀴 개별 (V)
  steer_pulse(pwm, ms)  조향 모터 펄스 (S). +255 = 오른쪽 (9/11 실측), 왼쪽은 -.
  stop()                X
"""
from __future__ import annotations
import glob, threading, time
import serial

def find_port():
    hits = sorted(glob.glob("/dev/serial/by-id/*Arduino*")) or sorted(glob.glob("/dev/ttyACM*"))
    return hits[0] if hits else None

class ArduinoV3:
    STEER_RIGHT_SIGN = +1          # S +pwm = 오른쪽 (9/11). 반대면 -1

    def __init__(self, port=None, baud=115200, cmd_hz=50.0):
        self.port = port or find_port()
        if not self.port: raise RuntimeError("아두이노 없음 (/dev/ttyACM*)")
        self.ser = serial.Serial(self.port, baud, timeout=0.05)
        time.sleep(2.3); self.ser.reset_input_buffer()
        self._lock = threading.Lock(); self._alive = True
        self._cmd: str | None = None; self._steer_until = 0.0; self._steer_cmd = ""
        self.last_rx = 0.0; self.wd_stops = 0; self.status = (0, 0, 0, 0)
        self.period = 1.0 / cmd_hz
        threading.Thread(target=self._tx, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()

    def ping(self, timeout=1.0):
        """v3 엔 P 가 없다. '?' 도움말로 생존 확인."""
        self.ser.reset_input_buffer(); self._raw("?")
        t = time.time() + timeout
        while time.time() < t:
            if self.last_rx and "v3" in self._banner: return True
            time.sleep(0.05)
        return False

    _banner = ""
    def _raw(self, line):
        try: self.ser.write((line + "\n").encode("ascii"))
        except Exception: pass

    def drive(self, pwm: int):
        with self._lock: self._cmd = f"D {int(max(-255, min(255, pwm)))}"
    def drive_lr(self, l: int, r: int):
        with self._lock: self._cmd = f"V {int(max(-255,min(255,l)))} {int(max(-255,min(255,r)))}"
    def steer_pulse(self, pwm: int, ms: int = 100):
        """조향 펄스. TX 스레드가 ms 동안 20Hz 로 반복 전송(워치독 대비)."""
        with self._lock:
            self._steer_cmd = f"S {int(max(-255,min(255,pwm)) * self.STEER_RIGHT_SIGN)} {int(ms)}"
            self._steer_until = time.time() + ms / 1000.0
    def stop(self):
        with self._lock: self._cmd = None; self._steer_until = 0.0
        self._raw("X")

    def _tx(self):
        while self._alive:
            with self._lock:
                c = self._cmd; sc = self._steer_cmd if time.time() < self._steer_until else None
            if c: self._raw(c)
            if sc: self._raw(sc)
            time.sleep(self.period)

    def _rx(self):
        while self._alive:
            try: line = self.ser.readline().decode("ascii", "ignore").strip()
            except Exception: continue
            if not line: continue
            self.last_rx = time.time()
            if line.startswith("ST "):
                p = line.split()
                if len(p) == 5:
                    try: self.status = tuple(int(x) for x in p[1:])
                    except ValueError: pass
            elif line.startswith("WD STOP"): self.wd_stops += 1
            elif "CAR v3" in line: self._banner = line

    def close(self):
        self._alive = False; self.stop(); time.sleep(0.05)
        try: self.ser.close()
        except Exception: pass
