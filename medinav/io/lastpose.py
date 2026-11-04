"""마지막 위치 기억(out/last_pose.json) — 재시작할 때마다 위치를 다시 잡지 않도록.

매핑·주행이 끝날 때 지도 이름과 함께 저장하고, 같은 지도로 주행을 시작하면 그 자리에서 출발한다.
지도 이름이 다르면 무시한다.
"""
import json
import os
import time

LAST_POSE_PERIOD_S = 5.0     # control_step 저장 주기 — 제어 루프 예산 잠식 방지
LAST_POSE_MIN_LOC = 0.2      # loc 신뢰도 이 미만이면 저장 스킵 — 틀린 위치를 기억하지 않기


def save_last_pose(path, x, y, th, map_name):
    """마지막 위치 원자적 저장(tmp 후 os.replace) — 로깅처럼 가볍게 json 한 줄."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(json.dumps({"x": float(x), "y": float(y), "th": float(th),
                            "map": map_name, "t": time.time()}))
    os.replace(tmp, path)


def load_last_pose(path, map_name):
    """저장된 마지막 위치 (x,y,th). map basename 불일치/파일 없음/깨짐 → None(기존 기본값)."""
    try:
        with open(path) as f:
            d = json.load(f)
        if d.get("map") != map_name:
            return None
        return (float(d["x"]), float(d["y"]), float(d["th"]))
    except Exception:
        return None


def loc_ok_to_save(lc):
    """저장 게이트: loc 신뢰도 낮음(<LAST_POSE_MIN_LOC) 또는 미산출(None)이면 스킵."""
    return lc is not None and lc >= LAST_POSE_MIN_LOC
