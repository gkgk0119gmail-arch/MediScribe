#!/usr/bin/env bash
# install_imu_service.sh — D435i 자이로를 부팅 때마다 준비하는 서비스를 등록한다 (sudo 필요).
#   1) system/build_kmods.sh --install  을 먼저 해 hid-sensor 모듈을 설치해 둘 것
#   2) sudo bash system/install_imu_service.sh
# 카메라를 부팅 후에 꽂았다면:  sudo systemctl restart d435i-imu.service
set -euo pipefail
cd "$(dirname "$0")"
install -m755 d435i-imu-setup.sh /usr/local/sbin/d435i-imu-setup.sh
install -m644 d435i-imu.service /etc/systemd/system/d435i-imu.service
systemctl daemon-reload
systemctl enable --now d435i-imu.service
systemctl --no-pager status d435i-imu.service | head -5
