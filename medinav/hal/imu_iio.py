"""D435i 자이로 리더 — Linux iio 경로.

D435i 의 IMU 는 USB HID 센서라 이 젯슨(tegra 커널)의 librealsense 로는 읽히지 않는다.
대신 커널 hid-sensor 모듈로 /sys/bus/iio 에 노출된 gyro_3d 를 버퍼(/dev/iio:deviceN)로 읽어
GyroYaw 에 적분한다. 버퍼 설정(scan_elements·buffer enable)은 root 권한이 필요해 부팅 서비스가
setup() 을 호출하고(system/d435i-imu-setup.sh), 리더는 장치를 열어 읽기만 한다.

축값 형식은 커널이 scan_elements/*_type 으로 알려 준다(이 장치는 le:s16/32 — 32비트 칸에 16비트 값).
형식을 하드코딩하지 말고 매번 읽는다. 요 축·부호는 config.IMU_GYRO_AXIS/SIGN (scripts/diag_gyro_axes.py 로 실측).
"""
from __future__ import annotations
import glob, os, struct, threading, time
from .imu import GyroYaw
from ..config import IMU_GYRO_AXIS, IMU_GYRO_SIGN

GYRO_NAME = "gyro_3d"
YAW_AXIS = "xyz".index(IMU_GYRO_AXIS)   # x=0 y=1 z=2
YAW_SIGN = IMU_GYRO_SIGN                 # 반시계 = +
_SAMPLE = 24       # <iii(12) + pad(4) + s64 ts(8)
_TS_OFF = 16


def _find_gyro_dir():
    for d in glob.glob("/sys/bus/iio/devices/iio:device*"):
        try:
            if open(d + "/name").read().strip() == GYRO_NAME:
                return d
        except OSError:
            continue
    return None


def _devnode(sysdir):
    return "/dev/iio:device" + sysdir.rsplit("device", 1)[1]


def setup(sysdir=None):
    """버퍼 스트리밍 켜기 (root 권한 필요). 부팅 서비스에서 호출."""
    g = sysdir or _find_gyro_dir()
    if not g:
        raise RuntimeError("gyro_3d iio 장치 없음 (hid-sensor 모듈/USB리셋 확인)")
    def w(rel, val):
        with open(os.path.join(g, rel), "w") as f:
            f.write(str(val))
    w("buffer/enable", 0)
    for e in ("in_anglvel_x_en", "in_anglvel_y_en", "in_anglvel_z_en", "in_timestamp_en"):
        p = os.path.join(g, "scan_elements", e)
        if os.path.exists(p):
            w("scan_elements/" + e, 1)
    w("buffer/length", 128)
    w("buffer/enable", 1)
    return g


def _chan_fmt(sysdir, ch):
    """scan_elements/<ch>_type 을 해석한다. 예: 'le:s16/32>>0' → (signed, realbits=16, storagebits=32, shift=0).

    2026-11-04: 이 장치는 축값을 **32비트 칸에 16비트**로 담는다(le:s16/32). 예전 코드는 "<iii" 로
    32비트 전체를 읽어 위쪽 16비트 쓰레기(0x0001, 0xffff …)까지 값에 섞였다 → 정지 상태에서
    수백만 rad/s 가 나와 GyroYaw 초기화가 끝나지 않았고, 융합이 오류 없이 조용히 꺼져 있었다.
    형식은 커널이 정하므로 하드코딩하지 말고 매번 읽는다."""
    try:
        t = open(os.path.join(sysdir, "scan_elements", f"{ch}_type")).read().strip()
        endian, rest = t.split(":")
        sign = rest[0] == "s"
        real, rest = rest[1:].split("/")
        stor, shift = (rest.split(">>") + ["0"])[:2]
        return endian == "be", sign, int(real), int(stor), int(shift)
    except (OSError, ValueError):
        return False, True, 32, 32, 0                      # 읽기 실패 시 예전 가정(<i4)


def _decode(raw, fmt):
    be, sign, real, _stor, shift = fmt
    v = int.from_bytes(raw, "big" if be else "little", signed=False) >> shift
    v &= (1 << real) - 1
    if sign and v >= (1 << (real - 1)):
        v -= 1 << real
    return v


class IIOGyro:
    """iio gyro_3d 버퍼를 읽어 GyroYaw 에 적분."""

    def __init__(self, sysdir=None, gyro=None, axis=YAW_AXIS, sign=YAW_SIGN):
        self.g = sysdir or _find_gyro_dir()
        if not self.g:
            raise RuntimeError("gyro_3d iio 장치 없음")
        self.scale = float(open(os.path.join(self.g, "in_anglvel_scale")).read())
        self.axis, self.sign = axis, sign
        self._fmt = [_chan_fmt(self.g, c) for c in ("in_anglvel_x", "in_anglvel_y", "in_anglvel_z")]
        self.gyro = gyro or GyroYaw()
        self._lock = threading.Lock()
        self._alive = True
        # 버퍼가 안 켜져 있으면 켜본다(권한 있으면)
        try:
            if open(os.path.join(self.g, "buffer/enable")).read().strip() != "1":
                setup(self.g)
        except OSError:
            pass
        self.fd = os.open(_devnode(self.g), os.O_RDONLY)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        import select
        buf = b""
        while self._alive:
            try:
                r, _, _ = select.select([self.fd], [], [], 0.2)
                if not r:
                    continue
                chunk = os.read(self.fd, _SAMPLE * 64)
            except (OSError, ValueError):
                break                                    # stop() 가 fd 닫음 — 조용히 종료
            if not chunk:
                continue
            buf += chunk
            while len(buf) >= _SAMPLE:
                rec, buf = buf[:_SAMPLE], buf[_SAMPLE:]
                st = self._fmt[0][3] // 8                 # 축당 저장 바이트 (이 장치는 4)
                vals = [_decode(rec[i * st:(i + 1) * st], self._fmt[i]) for i in range(3)]
                ts = struct.unpack_from("<q", rec, _TS_OFF)[0] / 1e9   # ns→s (하드웨어 시각)
                rate = self.sign * vals[self.axis] * self.scale
                with self._lock:
                    self.gyro.add(ts, rate, time.time())

    # ---- 소비자 API ----
    def set_stationary(self, flag):
        with self._lock:
            self.gyro.set_stationary(flag)

    def rate(self):
        with self._lock:
            return self.gyro.rate()

    def delta_yaw(self, t0, t1):
        with self._lock:
            return self.gyro.delta(t0, t1)

    def bias_dps(self):
        with self._lock:
            return self.gyro.bias * 57.2958

    def stop(self):
        self._alive = False
        try:
            os.close(self.fd)
        except OSError:
            pass


def open_iio_gyro():
    """(IIOGyro|None, 이유). 실패는 정상경로 — 소비자는 None 이면 융합 없이 동작."""
    try:
        d = _find_gyro_dir()
        if not d:
            return None, "gyro_3d iio 장치 없음 (모듈/USB리셋 필요)"
        return IIOGyro(d), f"iio {GYRO_NAME} @ {_devnode(d)} (axis={YAW_AXIS} sign={YAW_SIGN:+.0f})"
    except Exception as e:
        return None, f"iio 자이로 열기 실패({e})"
