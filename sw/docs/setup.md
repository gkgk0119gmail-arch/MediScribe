# 설치 (Jetson Orin NX · JetPack 6.x)

모든 명령은 저장소의 **`sw/`** 에서 실행한다 (`cd sw`).

검증 환경: Jetson Orin NX 16GB, JetPack 6.2 (L4T R36.4.3, 커널 5.15.148-tegra), Python 3.10.

## 1. 파이썬 패키지

```bash
pip3 install --user -r requirements.txt
sudo usermod -aG dialout $USER     # 라이다·아두이노 시리얼 권한 (다시 로그인)
```

## 2. 커널 모듈 — 라이다 어댑터, 조이스틱, IMU

젯슨 기본 커널에는 STL-27L 의 USB-시리얼 칩(PL2303), F710 조이스틱(xpad), D435i IMU 용 hid-sensor 드라이버가 빠져 있다.
같은 버전의 커널 소스를 kernel.org 에서 받아 빌드한다.

```bash
bash system/build_kmods.sh              # 받기 + 빌드 (build/kmods/)
sudo bash system/build_kmods.sh --install
```

확인: 라이다를 꽂으면 `/dev/ttyUSB0`, 조이스틱 수신기를 꽂으면 `/dev/input/js0` 가 생긴다.
F710 뒷면 스위치는 **X**(XInput) 에 둔다. 커널을 업데이트하면 다시 빌드해야 한다.

## 3. IMU 부팅 서비스

D435i 의 자이로는 부팅 때 카메라 펌웨어를 한 번 재시작해야 정상 데이터를 준다. 서비스가 이를 자동으로 한다.

```bash
sudo bash system/install_imu_service.sh
```

⚠️ 카메라를 **부팅한 뒤에** 꽂았다면 `sudo systemctl restart d435i-imu.service` 를 해야 자이로 데이터가 나온다.

## 4. 장착 확인 (한 번)

```bash
python3 -m medinav.hal.lidar_stl27l --seconds 5     # 라이다: 10 Hz, 스캔당 ~2,170점, CRC 오류 0% 면 정상
python3 -u scripts/diag_gyro_axes.py                 # 자이로: 반시계 90° 돌려 요 축·부호 확인 → medinav/config.py
python3 scripts/teleop_joy.py --dry                  # 조이스틱 입력만 확인 (모터 없음)
python3 scripts/teleop_joy.py --max 120              # 바퀴를 띄우고 전진/후진/조향 방향 확인
```

라이다 정면이 맞지 않으면 `medinav/config.py` 의 `LIDAR_FLIP`, `ANGLE_OFFSET_DEG` 를 조정한다.
차체 크기가 다르면 `BODY_BOX_M`(차체 반사 제거 상자)과 주행 인자 `--car-front --car-rear --car-halfw` 를 바꾼다.

## 5. 안전

- **물리 비상정지**(모터 전원 차단)를 손 닿는 곳에 둔다. 아두이노 워치독은 "명령 끊김"만 잡는다.
- F710 이 버튼 눌림 상태로 굳은 적이 있다(손을 대지 않았는데 RB 가 눌린 것으로 읽힘). 이러면 데드맨이 듣지 않는다 →
  주행 전 `teleop_joy.py --dry` 화면에서 ○RB(놓임)인지 확인하고, 이상하면 수신기를 뽑았다 꽂는다.
- 젯슨 전원선이 주행 중 당겨지지 않게 고정한다. 전원이 끊기면 그 자리에서 꺼진다(기록 없이).
