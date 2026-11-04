#!/bin/bash
# D435i IMU(iio) 부팅 셋업 — 재부팅 후 자이로를 자동으로 살린다 (system/install_imu_service.sh 가 등록).
# 순서: 모듈 로드 -> ★카메라 펌웨어 리부트(hardware_reset) -> 재열거 대기 -> 트리거+버퍼 -> 노드 권한
#
# 왜 hardware_reset 인가 (2026-09-20 실측): USB 재열거(authorized/unbind)만으로는 모션모듈 펌웨어가
# stale 상태로 남아 hid-sensor-hub 가 리포트 ID 를 못 찾음("No report with id 0xffffffff") -> accel/gyro 전부 0.
# librealsense hardware_reset() 으로 펌웨어를 리부트해야 모션모듈이 깨끗한 리포팅 상태가 되고
# 그 뒤 커널 hid-sensor 가 정상 데이터(accel 중력, gyro ~400Hz)를 받는다.
KH=${HIDIMU_DIR:-/opt/medinav/hidimu}   # build_kmods.sh --install 로 설치했으면 modprobe 가 먼저 잡는다
modprobe industrialio-triggered-buffer 2>/dev/null || true
modprobe kfifo_buf 2>/dev/null || true
for m in hid-sensor-hub hid-sensor-iio-common hid-sensor-trigger hid-sensor-gyro-3d hid-sensor-accel-3d; do
  modprobe $m 2>/dev/null || insmod $KH/$m.ko 2>/dev/null || true
done
sleep 3   # 부팅 직후 USB 안정화

# ★핵심: 카메라 펌웨어 리부트 (모션모듈 초기화). root 로도 import 되는 pyrealsense2 가 필요
python3 - <<'PY'
try:
    import pyrealsense2 as rs
    for d in rs.context().query_devices():
        if "D435" in d.get_info(rs.camera_info.name):
            d.hardware_reset()
            print("hardware_reset sent")
except Exception as e:
    print("hardware_reset skip:", e)
PY
sleep 12   # 펌웨어 리부트 + USB 재열거 + usbhid/hid-sensor-hub 재바인딩 대기

# gyro_3d: 이름으로 탐색(장치번호 무관) -> 버퍼 끄고 -> 트리거 바인딩 -> scan elements -> 버퍼 켜기
for g in /sys/bus/iio/devices/iio:device*; do
  if [ "$(cat $g/name 2>/dev/null)" = "gyro_3d" ]; then
    idx=${g##*device}
    echo 0 > "$g/buffer/enable" 2>/dev/null
    for t in /sys/bus/iio/devices/trigger*; do
      if [ "$(cat $t/name 2>/dev/null)" = "gyro_3d-dev$idx" ]; then
        echo "gyro_3d-dev$idx" > "$g/trigger/current_trigger" 2>/dev/null
      fi
    done
    for e in in_anglvel_x_en in_anglvel_y_en in_anglvel_z_en in_timestamp_en; do
      [ -e "$g/scan_elements/$e" ] && echo 1 > "$g/scan_elements/$e"
    done
    echo 128 > "$g/buffer/length"
    echo 1 > "$g/buffer/enable"
    chmod a+r "/dev/iio:device$idx" 2>/dev/null || true
  fi
done
exit 0
