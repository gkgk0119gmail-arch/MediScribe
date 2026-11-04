#!/usr/bin/env bash
# build_kmods.sh — 젯슨 기본 커널(L4T)에 빠져 있는 드라이버 3묶음을 같은 버전 커널 소스로 빌드한다.
#   pl2303   : STL-27L 라이다의 USB-시리얼 어댑터(Prolific 067b:2303) → /dev/ttyUSB0
#   xpad     : Logitech F710 (XInput 모드, 046d:c21f)                → /dev/input/js0
#   hidimu   : D435i IMU 용 hid-sensor 모듈(hub, iio-common, trigger, gyro-3d, accel-3d) → /sys/bus/iio
#
# 사용:  bash system/build_kmods.sh            # 받아서 빌드만 (build/kmods/ 에 .ko 생성)
#        sudo bash system/build_kmods.sh --install   # 빌드 + 커널 모듈 폴더에 복사 + depmod
# 필요: 커널 헤더(/lib/modules/$(uname -r)/build), 인터넷.
# 커널을 업데이트하면 다시 돌려야 한다.
set -euo pipefail
KREL=$(uname -r)                       # 예: 5.15.148-tegra
KVER=${KVER:-${KREL%%-*}}              # → 5.15.148 (커널 소스 태그)
KDIR=/lib/modules/$KREL/build
SRC="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain"
OUT=$(cd "$(dirname "$0")/.." && pwd)/build/kmods
[ -d "$KDIR" ] || { echo "커널 헤더가 없습니다: $KDIR"; exit 1; }
mkdir -p "$OUT"/{pl2303,xpad,hidimu}

get() { curl -fsSL "$SRC/$1?h=v$KVER" -o "$2" || { echo "받기 실패: $1"; exit 1; }; }
echo "[1/3] 소스 받기 (커널 v$KVER)"
get drivers/usb/serial/pl2303.c                         "$OUT/pl2303/pl2303.c"
get drivers/usb/serial/pl2303.h                         "$OUT/pl2303/pl2303.h"
get drivers/input/joystick/xpad.c                       "$OUT/xpad/xpad.c"
get drivers/hid/hid-sensor-hub.c                        "$OUT/hidimu/hid-sensor-hub.c"
get drivers/hid/hid-ids.h                               "$OUT/hidimu/hid-ids.h"
get drivers/iio/common/hid-sensors/hid-sensor-attributes.c "$OUT/hidimu/hid-sensor-attributes.c"
get drivers/iio/common/hid-sensors/hid-sensor-trigger.c "$OUT/hidimu/hid-sensor-trigger.c"
get drivers/iio/common/hid-sensors/hid-sensor-trigger.h "$OUT/hidimu/hid-sensor-trigger.h"
get drivers/iio/gyro/hid-sensor-gyro-3d.c               "$OUT/hidimu/hid-sensor-gyro-3d.c"
get drivers/iio/accel/hid-sensor-accel-3d.c             "$OUT/hidimu/hid-sensor-accel-3d.c"
# 커널 트리 안에서는 상대경로로 공용 헤더를 찾는다 → 한 폴더에 모았으니 같은 폴더를 보게 바꾼다
sed -i 's#"../common/hid-sensors/hid-sensor-trigger.h"#"hid-sensor-trigger.h"#' \
    "$OUT/hidimu/hid-sensor-gyro-3d.c" "$OUT/hidimu/hid-sensor-accel-3d.c"
echo "obj-m += pl2303.o" > "$OUT/pl2303/Makefile"
echo "obj-m += xpad.o"   > "$OUT/xpad/Makefile"
cat > "$OUT/hidimu/Makefile" <<'MK'
obj-m += hid-sensor-hub.o
obj-m += hid-sensor-iio-common.o
obj-m += hid-sensor-trigger.o
obj-m += hid-sensor-gyro-3d.o
obj-m += hid-sensor-accel-3d.o
hid-sensor-iio-common-y := hid-sensor-attributes.o
MK

echo "[2/3] 빌드"
for m in pl2303 xpad hidimu; do
  make -s -C "$KDIR" M="$OUT/$m" ARCH=arm64 modules
done
ls "$OUT"/*/*.ko

if [ "${1:-}" = "--install" ]; then
  echo "[3/3] 설치"
  D=/lib/modules/$KREL/kernel/drivers
  install -D -m644 "$OUT/pl2303/pl2303.ko" "$D/usb/serial/pl2303.ko"
  install -D -m644 "$OUT/xpad/xpad.ko"     "$D/input/joystick/xpad.ko"
  for f in "$OUT"/hidimu/*.ko; do install -D -m644 "$f" "$D/medinav-hidimu/$(basename "$f")"; done
  depmod -a
  echo "완료 — 라이다·조이스틱은 꽂으면 자동 로드, IMU 는 system/install_imu_service.sh 로 부팅 서비스 등록"
else
  echo "[3/3] 설치 생략 (--install 로 설치)"
fi
