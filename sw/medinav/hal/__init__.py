"""
hal — 하드웨어 추상화 계층.

nav 코드는 인터페이스(Lidar)와 변환 함수(hal.lidar)에만 의존한다.
구현: lidar_stl27l(STL-27L 직렬 파서), imu_iio(D435i 자이로, Linux iio), arduino_v3(모터 제어).
"""
from .base import Lidar

__all__ = ["Lidar"]
