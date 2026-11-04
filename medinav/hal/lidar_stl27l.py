"""
LDROBOT STL-27L 라이다 드라이버 (순수 파이썬, pyserial). ROS 없음.

프로토콜 (제조사 ROS2 드라이버 ldlidar_stl_ros2 의 lipkg.cpp 를 파이썬으로 옮김):
  패킷 47바이트 = 0x54, 0x2C, speed u16(deg/s), start_angle u16(0.01deg),
                 12 × (dist u16 mm, intensity u8), end_angle u16, timestamp u16(ms), crc8
  모두 리틀엔디언. CRC8 은 마지막 바이트를 제외한 46바이트에 대해 아래 테이블로.
  921600 bps. 각도는 라이다 기준 시계방향(위에서 봤을 때) 0~360°, 점 각도는 start→end 를 11등분.

출력 규약:
  iter_scans()/latest_scan(): 한 바퀴 = [(intensity, angle_deg, dist_mm), ...] 측정 순서 그대로
    → medinav.hal.lidar.scan_to_points / scan_to_rays 가 그대로 받는다.
  scan(): hal.base.Lidar 계약 (angles[rad, 로봇기준], ranges[m]) — config 의 LIDAR_FLIP/ANGLE_OFFSET 적용.

사용 (진단):  python3 -m medinav.hal.lidar_stl27l --seconds 5
  rpm, 스캔당 점수, CRC 오류율, 8방향 최소거리를 찍는다. 앞에 손을 대 보면 FLIP 부호와 장착 오프셋을 정할 수 있다.
"""
from __future__ import annotations

import glob
import math
import threading
import time

import numpy as np

HEADER, VER_LEN, PKT_LEN, PTS = 0x54, 0x2C, 47, 12
BAUD = 921600

_CRC = bytes([
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,
    0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,
    0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,
    0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,
    0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,
    0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8])


def crc8(buf: bytes) -> int:
    c = 0
    for b in buf:
        c = _CRC[(c ^ b) & 0xFF]
    return c


def find_port() -> str | None:
    """USB-시리얼 어댑터 자동 탐색. STL-27L 은 보통 CP2102 나 PL2303 브리지로 온다."""
    for pat in ("/dev/serial/by-id/*Prolific*", "/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon*",
                "/dev/serial/by-id/*CH340*", "/dev/serial/by-id/*USB_Serial*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else None


def parse_packet(pkt: bytes):
    """47바이트 → (speed_dps, [(intensity, angle_deg, dist_mm)]*12, timestamp_ms). CRC 불일치면 None."""
    if crc8(pkt[:-1]) != pkt[-1]:
        return None
    speed = pkt[2] | (pkt[3] << 8)
    start = (pkt[4] | (pkt[5] << 8)) / 100.0
    end = (pkt[42] | (pkt[43] << 8)) / 100.0
    ts = pkt[44] | (pkt[45] << 8)
    step = ((end - start) % 360.0) / (PTS - 1)
    pts = []
    for i in range(PTS):
        o = 6 + 3 * i
        dist = pkt[o] | (pkt[o + 1] << 8)
        inten = pkt[o + 2]
        pts.append((inten, (start + step * i) % 360.0, dist))
    return speed, pts, ts


class STL27L:
    """백그라운드 스레드가 시리얼을 읽어 한 바퀴씩 모은다."""

    def __init__(self, port: str | None = None, baud: int = BAUD, timeout: float = 0.5):
        import serial
        self.port = port or find_port()
        if not self.port:
            raise RuntimeError("라이다 시리얼 포트 없음 (/dev/ttyUSB*, /dev/serial/by-id/*)")
        self.ser = serial.Serial(self.port, baud, timeout=timeout)
        self._buf = bytearray()
        self._cur: list = []
        self._last_angle = -1.0
        self._latest: list | None = None
        self._seq = 0
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._stop = threading.Event()
        self.speed_dps = 0.0
        self.crc_errors = 0
        self.packets = 0
        self.last_ts = 0
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    # ---- 읽기 스레드 ----
    def _run(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting or 512))
            except Exception:
                time.sleep(0.05); continue
            if not chunk:
                continue
            self._buf += chunk
            self._drain()

    def _drain(self):
        b = self._buf
        i = 0
        n = len(b)
        while i + PKT_LEN <= n:
            if b[i] != HEADER or b[i + 1] != VER_LEN:
                i += 1; continue
            res = parse_packet(bytes(b[i:i + PKT_LEN]))
            if res is None:
                self.crc_errors += 1; i += 1; continue
            self.packets += 1
            speed, pts, ts = res
            self.speed_dps = speed; self.last_ts = ts
            self._push(pts)
            i += PKT_LEN
        del b[:i]

    def _push(self, pts):
        for p in pts:
            a = p[1]
            if self._last_angle >= 0 and a < self._last_angle - 180.0:   # 360→0 넘어감 = 한 바퀴 완료
                if len(self._cur) > 100:
                    with self._cv:
                        self._latest = self._cur; self._seq += 1; self._cv.notify_all()
                self._cur = []
            self._cur.append(p)
            self._last_angle = a

    # ---- 소비자 API ----
    def latest_scan(self):
        with self._lock:
            return self._latest

    def wait_scan(self, timeout=1.0):
        """다음 새 스캔을 기다려 반환 (없으면 None)."""
        with self._cv:
            seq = self._seq
            self._cv.wait_for(lambda: self._seq != seq, timeout)
            return self._latest if self._seq != seq else None

    def iter_scans(self, min_len=100):
        """한 바퀴 [(intensity, angle_deg, dist_mm), ...] 를 계속 낸다."""
        while not self._stop.is_set():
            s = self.wait_scan()
            if s is not None and len(s) >= min_len:
                yield s

    def scan(self):
        """hal.base.Lidar 계약: (angles[rad, 로봇기준], ranges[m]). config 의 FLIP/오프셋 적용."""
        from .lidar import scan_to_rays
        s = self.wait_scan()
        if s is None:
            return np.zeros(0), np.zeros(0)
        return scan_to_rays(s)

    @property
    def rpm(self):
        return self.speed_dps / 6.0

    def stop(self):
        self._stop.set()

    def close(self):
        self.stop()
        try: self.ser.close()
        except Exception: pass


def _main():
    import argparse
    ap = argparse.ArgumentParser(description="STL-27L 진단")
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=5)
    a = ap.parse_args()
    ld = STL27L(a.port)
    print(f"포트 {ld.port} @ {BAUD}")
    t0 = time.time(); n = 0; last = None
    while time.time() - t0 < a.seconds:
        s = ld.wait_scan(1.0)
        if s is None:
            print("  스캔 없음 (1초)… 패킷", ld.packets, "CRC오류", ld.crc_errors); continue
        n += 1; last = s
    ld.close()
    if not last:
        print("스캔을 한 바퀴도 못 모았습니다. 배선/보레이트 확인."); return
    d = np.array([p[2] for p in last], dtype=float) / 1000.0
    ang = np.array([p[1] for p in last])
    valid = d > 0
    print(f"{a.seconds:.0f}초: 스캔 {n}회 ({n/a.seconds:.1f}Hz), rpm {ld.rpm:.0f}, 스캔당 점 {len(last)}, 유효 {valid.mean()*100:.0f}%, "
          f"패킷 {ld.packets}, CRC오류 {ld.crc_errors} ({ld.crc_errors/max(1,ld.packets+ld.crc_errors)*100:.2f}%)")
    print(f"거리 min {d[valid].min():.2f}m  max {d[valid].max():.2f}m  중앙값 {np.median(d[valid]):.2f}m")
    print("8방향 최소거리 (라이다 원시 각도 기준, 0°=라이다 표기 기준 앞):")
    for k in range(8):
        lo, hi = k * 45 - 22.5, k * 45 + 22.5
        m = valid & (((ang - k * 45 + 180) % 360) - 180 > -22.5) & (((ang - k * 45 + 180) % 360) - 180 <= 22.5)
        print(f"  {k*45:3d}°: {d[m].min():.2f}m" if m.any() else f"  {k*45:3d}°: -")


if __name__ == "__main__":
    _main()
