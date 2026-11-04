"""HAL 인터페이스 정의."""
from __future__ import annotations
from abc import ABC, abstractmethod
import numpy as np


class Lidar(ABC):
    """2D 라이다 계약."""

    @abstractmethod
    def scan(self) -> tuple[np.ndarray, np.ndarray]:
        """반환: (angles[rad, 로봇 기준], ranges[m])."""
